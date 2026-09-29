// Swing-up a energia del pendolo di Furuta.
// Riferimenti: README §7 (legge di swing-up) e §8 (da thetaDDot a coppia/V).
//
// Catena di controllo implementata nel loop():
//   1) errE -> thetaDDot_cmd        EnergySwingUpController::compute()
//   2) richiamo al centro + integratore -> riferimento di velocita' del braccio
//   3) thetaDDot -> tau -> i -> V   feed-forward dal modello
//   4) PI di velocita' + modello   SimpleFOC, comando totale limitato a 2 V
// In BALANCE il passo 2 usa l'LQR della libreria (SwingSequence), che cattura
// e stabilizza la verticale; 3) e 4) restano invariati.
//
// La catena di conversione (README §8, percorso A) e' in libreria:
//   FurutaDynamics::torqueForArmAcceleration(...)   [dynamics.hpp]
//   Motor::currentForTorque(tau)                    [motor.hpp]
//   Motor::voltageForCurrent(i, thetaDot)           [motor.hpp]
// con R = ActuatorLimits::phaseResistance [parameters.hpp].
// Restano da identificare maxTorque/maxCurrent (oggi vuoti): la saturazione
// e' quindi solo il limite di tensione dello sketch, non un limite di libreria.

#include <furuta.hpp>
// dynamics.hpp usa Eigen (ArduinoEigen): incluso esplicitamente per non
// appesantire furuta.hpp per gli sketch che non ne hanno bisogno.
// energy_swing_up.hpp e swing_sequence.hpp sono esposte da furuta.hpp.
#include <furuta/dynamics.hpp>
#include <SimpleFOC.h>
#include <cstdio>

// ===========================================================================
// Configurazione
// ===========================================================================

static constexpr uint32_t PERIOD_US = 1000;
static constexpr float FS_HZ = 1.0e6f / PERIOD_US; // nominale; il filtro usa il dt misurato
static constexpr float    FC_HZ      = 20.0f;      // taglio passa-basso

// Pendolo appeso: entro 15 gradi dal basso un angolo fermo per 50 ms viene
// riportato a zero (rest-lock dello stimatore), cosi' il dither di
// quantizzazione non appare come velocita'. Non tocca SWING/BALANCE, dove
// alpha e' vicino a pi e il filtro resta a banda piena.
static constexpr float REST_HOLD_S = 0.05f;
static constexpr float REST_ZONE_RAD = 15.0f * static_cast<float>(M_PI) / 180.0f;

// Offset meccanico del pendolo: alpha = 0 con il pendolo appeso (E = 0).
// Lo zero dell'AS5600 del pendolo e' ruotato di 180 gradi (da verificare:
// pendolo appeso fermo -> ErrE ~= E* = 2*m*g*l ~= 0.0279 J; se ErrE ~= 0
// l'offset e' sbagliato di pi).
static constexpr double ALPHA_OFFSET = M_PI;

// Una lettura AS5600 fallita non e' segnalata da SimpleFOC (getRawCount non
// controlla l'esito I2C): in caso di NACK tornano 0xFF/0xFF, cioe' un angolo a
// ~2pi in un solo campione. Il Kalman lo assorbe e produce un transiente di
// velocita' che l'LQR amplifica in uno scatto dell'arm. Un salto oltre questo
// limite e' impossibile per il pendolo: si tiene l'ultimo campione valido.
static constexpr double ALPHA_MAX_STEP = 0.2;  // rad per campione (~230 rad/s a 0.86 ms)
static constexpr double THETA_OFFSET = M_PI; // stesso zero meccanico di furuta.ino

// Segno di alpha nella convenzione del modello. La terna del modello e' fissata
// dal verso positivo del motore (theta = theta_FOC); il verso positivo di alpha
// dipende dall'orientamento della cerniera e dal montaggio dell'AS5600 del
// pendolo. Con lo swing-up attivo: se il motore smorza invece di pompare
// (ErrE cresce invece di calare), invertire questo segno.
static constexpr double swingDirection = -1.0;

// Profilo di homing come in furuta.ino.
static constexpr float HOMING_P_ANGLE = 8.0f;
static constexpr float HOMING_VELOCITY_LIMIT = 2.0f;

// Bus I2C: braccio sul bus Wire (1, 2), pendolo sul bus Wire1 (41, 42).
// Due AS5600 hanno lo stesso indirizzo fisso (0x36), quindi servono due bus.
static constexpr int ARM_SDA = 1;
static constexpr int ARM_SCL = 2;
static constexpr int PENDULUM_SDA = 41;
static constexpr int PENDULUM_SCL = 42;

// Motore: 8 poli -> 4 coppie polari; 3 PWM su GPIO 6/5/4 (come furuta.ino).
static constexpr int MOTOR_POLE_PAIRS = 4;
static constexpr int PIN_PWM_A = 6;
static constexpr int PIN_PWM_B = 5;
static constexpr int PIN_PWM_C = 4;

// Il target energetico decide l'ampiezza; k corregge l'errore di energia.
// maxArmAcceleration limita l'intensita' delle pompate.
// TARGET_ENERGY_FRACTION > 1 e' deliberato: con un target pari a E* la pompa
// si spegne (errE -> 0) prima del top e qualunque perdita ferma il pendolo
// sotto la verticale, fuori dal corridoio di cattura dell'LQR (in
// simulazione con 0.95 la cattura non arriva nemmeno con attrito nullo).
// A 1.10 il pendolo attraversa il top e l'LQR cattura in ~2-3.5 s con
// b2 <= 5e-5 N*m*s/rad; con b2 >= 1e-4 serve piu' autorita' di pompaggio.
// I valori d'attrito sono ipotesi di test, non un'identificazione del banco.
// Questi tempi vanno verificati sul banco; cattura e mantenimento della
// verticale sono affidati all'LQR della libreria (lqr.hpp).
static constexpr double SWING_UP_GAIN = 500.0;
static constexpr double MAX_ARM_ACCELERATION = furuta::SwingSequence::maxAcceleration;
static constexpr double TARGET_ENERGY_FRACTION = 1.25;

// Il modello da solo non compensa l'attrito statico osservato sul banco.
// Il PI gia' usato nell'homing corregge l'errore fra velocita' richiesta e reale.
// Ogni contributo (modello e PI) dispone di meta' della tensione: la somma
// resta entro SWING_VOLTAGE_LIMIT anche con il feed-forward di SimpleFOC.
static constexpr double MAX_ARM_VELOCITY = furuta::SwingSequence::maxVelocity;
static constexpr float SWING_VOLTAGE_LIMIT = 2.0f;

// ===========================================================================
// Oggetti
// ===========================================================================

furuta::ActuatorLimits actuatorParams{};
furuta::PinwheelParameters pinwheel{};
furuta::ArmParameters arm{};
furuta::FrictionParameters friction{};
furuta::FurutaDynamics dynamics{pinwheel, arm, friction};
furuta::Motor motorModel{actuatorParams};
furuta::EnergySwingUpController swingUp{
    arm, furuta::EnergySwingUpConfig{
             SWING_UP_GAIN, MAX_ARM_ACCELERATION, TARGET_ENERGY_FRACTION}};
furuta::SensorData s{};
furuta::AS5600VelocityEstimator alphaEstimator{
    furuta::VelocityFilterMode::Kalman, FC_HZ, FS_HZ,
    1.0e-6f, 1.0e-2f, 0.5f, 0.1667f, REST_HOLD_S, REST_ZONE_RAD};
uint32_t lastAlphaSampleUs = 0;
double lastGoodAlpha = 0.0;
bool haveGoodAlpha = false;
uint32_t alphaGlitchCount = 0;
uint32_t burstSeenGlitches = 0;
furuta::SwingSequence sequence;
double thetaHome = 0.0; // setpoint FOC assoluto; theta di lavoro = shaftAngle - thetaHome

MagneticSensorI2C armSensor = MagneticSensorI2C(AS5600_I2C);       // theta (bus Wire)
MagneticSensorI2C pendulumSensor = MagneticSensorI2C(AS5600_I2C);  // alpha (bus Wire1)
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C);
Commander command = Commander(Serial);

// ===========================================================================
// Setup
// ===========================================================================

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Sensori: braccio sul bus Wire, pendolo sul bus Wire1 (stesso indirizzo I2C).
  Wire.begin(ARM_SDA, ARM_SCL);
  armSensor.init();
  // 400 kHz: le due letture AS5600 per ciclo sono il costo dominante del
  // loop; a 100 kHz il periodo reale resta ~2 ms e l'LQR e' al limite di
  // stabilita' (ciclo limite e caduta). Le AS5600 supportano 400 kHz.
  Wire.setClock(400000);
  Wire1.begin(PENDULUM_SDA, PENDULUM_SCL);
  pendulumSensor.init(&Wire1);
  Wire1.setClock(400000);

  // Il motore usa l'AS5600 del braccio per la commutazione FOC.
  motor.linkSensor(&armSensor);

  // driver config
  // power supply voltage [V]: alimentatore da banco a 24 V (ActuatorLimits::voltage)
  driver.voltage_power_supply = actuatorParams.voltage;
  driver.init();
  motor.linkDriver(&driver);

  // Prima homing in posizione, poi swing con anello interno di velocita'.
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::voltage;
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 1000.0f; // profilo homing di furuta.ino
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = HOMING_P_ANGLE;
  motor.velocity_limit = HOMING_VELOCITY_LIMIT;
  motor.voltage_limit = 6.0f; // homing come in furuta.ino; poi limite a 2 V

  motor.useMonitoring(Serial);

  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  thetaHome = furuta::homeTarget(motor.sensor_direction, THETA_OFFSET);
  motor.PID_velocity.reset();
  // Il limite e' in volt: non e' una misura o una regolazione della corrente.
  Serial.printf("SwingUp homing+wait+center: gain=%.0f,maxA=%.1f,targetE=%.2f,maxW=%.1f,maxV=%.1f,thetaLimit=+/-2pi\n",
      SWING_UP_GAIN, MAX_ARM_ACCELERATION, TARGET_ENERGY_FRACTION,
      MAX_ARM_VELOCITY, static_cast<double>(SWING_VOLTAGE_LIMIT));

  // Aggancia lo stimatore alla lettura corrente: il primo campione non
  // genera uno spike di velocita'.
  s.alpha = swingDirection * furuta::referenceAngle(pendulumSensor.getSensorAngle(), ALPHA_OFFSET);
  lastGoodAlpha = s.alpha;
  haveGoodAlpha = true;
  alphaEstimator.reset(s.alpha);
  lastAlphaSampleUs = micros();
}

// ===========================================================================
// Loop
// ===========================================================================

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  motor.loopFOC();

  // Pendolo: alpha = 0 appeso + stima filtrata di alphaDot.
  const uint32_t readStartUs = micros();
  const double measuredAlpha =
      swingDirection * furuta::referenceAngle(pendulumSensor.getSensorAngle(), ALPHA_OFFSET);
  const uint32_t readPeriodUs = micros() - readStartUs;
  if (haveGoodAlpha
      && std::abs(std::remainder(measuredAlpha - lastGoodAlpha, 2.0 * M_PI)) > ALPHA_MAX_STEP) {
    ++alphaGlitchCount;
  } else {
    lastGoodAlpha = measuredAlpha;
    haveGoodAlpha = true;
    s.alpha = measuredAlpha;
  }
  const uint32_t alphaSampleUs = micros();
  const uint32_t samplePeriodUs = alphaSampleUs - lastAlphaSampleUs;
  lastAlphaSampleUs = alphaSampleUs;
  s.alphaDotFiltered = alphaEstimator.update(s.alpha, samplePeriodUs * 1.0e-6f);
  // Il periodo medio distingue un loop lento da uno veloce con picchi: a
  // >1.5 ms l'LQR perde margine e va in ciclo limite.
  static uint32_t maxSamplePeriodUs = 0, samplePeriodSumUs = 0, readPeriodSumUs = 0;
  static uint32_t sampleCount = 0;
  samplePeriodSumUs += samplePeriodUs;
  readPeriodSumUs += readPeriodUs;
  ++sampleCount;
  if (samplePeriodUs > maxSamplePeriodUs) maxSamplePeriodUs = samplePeriodUs;

  // Braccio: thetaDot dal sensore FOC (sensor_direction gia' applicato da
  // SimpleFOC), quindi nello stesso verso positivo della coppia comandata.
  // motor.shaft_velocity e' aggiornato da move() al giro precedente (~1 ms).
  s.thetaDotFiltered = motor.shaft_velocity;
  // Angolo continuo, senza wrap/clamp: un giro oltre il limite deve restare visibile.
  s.theta = motor.shaftAngle() - thetaHome;

  // --- catena di controllo -------------------------------------------------
  // 1) legge di swing-up: errE -> thetaDDot_cmd (saturata su maxArmAcceleration)
  const furuta::controlOutput cmd = swingUp.compute(s);

  const double dt = samplePeriodUs * 1.0e-6;
  const auto previousPhase = sequence.phase;
  const bool wasVelocityControl = sequence.controlsVelocity();
  sequence.update(s, cmd.armAcceleration, dt);
  // SWING e BALANCE condividono l'anello di velocita': in SWING il riferimento
  // arriva dalla legge energetica, in BALANCE dall'LQR della libreria.
  const bool velocityControl = sequence.controlsVelocity();
  if (sequence.phase != previousPhase) {
    motor.voltage_limit = SWING_VOLTAGE_LIMIT;
    // Swing <-> Balance non riconfigura il PI: integrale e riferimento di
    // velocita' si conservano alla cattura e alla perdita d'equilibrio.
    if (velocityControl != wasVelocityControl) {
      motor.controller = velocityControl ? MotionControlType::velocity : MotionControlType::angle;
      motor.velocity_limit = velocityControl ? MAX_ARM_VELOCITY : HOMING_VELOCITY_LIMIT;
      motor.PID_velocity.limit = velocityControl ? SWING_VOLTAGE_LIMIT / 2.0f : SWING_VOLTAGE_LIMIT;
      motor.PID_velocity.output_ramp = 100.0f;
      motor.LPF_velocity.Tf = 0.005f;
      motor.PID_velocity.reset();
    }
    // Il log del banco (29/09) mostra che il hold a centro con il PI di
    // velocita' va in stick-slip: l'integrale carica contro l'attrito statico
    // (V PI costante ~0.34 V), poi slitta e il kick eccita il pendolo
    // (ThetaDot ~10 rad/s, AlphaDot ~-12 rad/s) che rimbalza sull'arm. In
    // attesa e assestamento il driver viene quindi disabilitato, come in
    // furuta.ino: il pendolo si smorza da solo e il braccio resta libero.
    // Il rientro al centro (RETURNING) e lo swing restano abilitati.
    if (sequence.phase == furuta::SwingSequence::Phase::Settling
        || sequence.phase == furuta::SwingSequence::Phase::Waiting) {
      motor.disable();
    } else {
      motor.enable();
    }
  }

  // 2) thetaDDot_cmd -> tau (SWING e BALANCE): inversa del modello, tau2 = 0
  const double tau = velocityControl ? dynamics.torqueForArmAcceleration(
      s.alpha, s.thetaDotFiltered, s.alphaDotFiltered, sequence.acceleration) : 0.0;

  // 3) tau -> i -> V (percorso A in libreria: Motor::currentForTorque/voltageForCurrent)
  const double current = motorModel.currentForTorque(tau);
  const double modelVoltage = velocityControl ? furuta::clamp(
      motorModel.voltageForCurrent(current, s.thetaDotFiltered),
      -SWING_VOLTAGE_LIMIT / 2.0, SWING_VOLTAGE_LIMIT / 2.0) : 0.0;

  // 4) Il PI aumenta lo sforzo se il braccio non segue il riferimento.
  motor.feed_forward_voltage.q = modelVoltage;
  // SWING e BALANCE seguono il riferimento di velocita'; fuori (homing,
  // rientro) il braccio viene riportato allo zero in posizione.
  motor.move(velocityControl ? sequence.velocityTarget : thetaHome);
  // Con il driver disabilitato move() esce prima di aggiornare current_sp e
  // shaft_velocity_sp: in diagnostica vanno azzerati per non mostrare valori
  // congelati nell'attesa (State SETTLING/WAIT_PUSH).
  const double voltage = motor.enabled
      ? motor.current_sp + motor.feed_forward_voltage.q : 0.0;
  const double velocityTarget = motor.enabled ? motor.shaft_velocity_sp : 0.0;

  // --- burst sul tick in BALANCE -------------------------------------------
  // Cattura 128 campioni (~100 ms) su uno scatto (|alphaDot| > 0.5 dopo
  // 0.5 s di BALANCE) o su una lettura pendolo rifiutata dal gate
  // (alphaGlitchCount), poi li scarica senza bloccare il loop. Colonne:
  // alpha, alphaDot, theta, thetaDot, target, ThetaDDot, V modello, V PI,
  // alpha misurata pre-gate, periodo campione us. L'ordine distingue
  // stick-slip dell'arm, disturbo del pendolo e guasto di lettura.
  static constexpr int BURST_SAMPLES = 128;
  static float burst[BURST_SAMPLES][10];
  static int burstFill = -1;  // -1 inattivo, 0..N cattura, N = da scaricare
  static int burstSent = 0;
  static unsigned long balanceSince = 0;
  if (previousPhase != furuta::SwingSequence::Phase::Balance
      && sequence.phase == furuta::SwingSequence::Phase::Balance) {
    balanceSince = millis();
  }
  if (burstFill < 0 && sequence.phase == furuta::SwingSequence::Phase::Balance
      && millis() - balanceSince > 500
      && (std::abs(s.alphaDotFiltered) > 0.5 || alphaGlitchCount != burstSeenGlitches)) {
    burstFill = 0;
    burstSent = 0;
    burstSeenGlitches = alphaGlitchCount;
  }
  if (burstFill >= 0 && burstFill < BURST_SAMPLES) {
    float* row = burst[burstFill];
    row[0] = s.alpha;
    row[1] = s.alphaDotFiltered;
    row[2] = s.theta;
    row[3] = s.thetaDotFiltered;
    row[4] = velocityTarget;
    row[5] = sequence.acceleration;
    row[6] = modelVoltage;
    row[7] = motor.current_sp;
    row[8] = static_cast<float>(measuredAlpha);
    row[9] = static_cast<float>(samplePeriodUs);
    ++burstFill;
  }
  if (burstFill == BURST_SAMPLES && burstSent < BURST_SAMPLES
      && Serial.availableForWrite() >= 96) {
    char burstLine[128];
    const float* row = burst[burstSent];
    const int n = std::snprintf(burstLine, sizeof(burstLine),
        "BURST,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.0f\n",
        burstSent, row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7],
        row[8], row[9]);
    if (n > 0 && n < static_cast<int>(sizeof(burstLine))) {
      Serial.write(reinterpret_cast<const uint8_t*>(burstLine), n);
      ++burstSent;
    }
  }
  if (burstFill == BURST_SAMPLES && burstSent == BURST_SAMPLES) {
    burstFill = -1;
    burstSent = 0;
  }

  // --- diagnostica seriale (CSV: separatore ",") ---------------------------
  // Una riga ogni 100 ms, trasmessa a piccoli blocchi solo se c'e' spazio.
  // Le Serial.print consecutive bloccavano FOC e controllo per ~14 ms.
  static unsigned long last_print = 0;
  static char diagnostics[384];
  static size_t diagnosticsLength = 0;
  static size_t diagnosticsSent = 0;
  if (diagnosticsSent == diagnosticsLength && millis() - last_print >= 100) {
    last_print = millis();
    const int length = std::snprintf(diagnostics, sizeof(diagnostics),
        "State: %s,Alpha: %.4f,AlphaDot stimata: %.4f,Theta: %.4f,ThetaDot: %.4f,"
        "ThetaDot target: %.4f,ErrE: %.6f,ThetaDDot: %.3f,Tau modello: %.5f,"
        "V modello: %.3f,V PI: %.3f,V: %.3f,Theta ref LQR: %.4f,"
        "dt medio us: %lu,lettura pendolo us: %lu,dt max us: %lu,glitch: %lu\n",
        sequence.name(), s.alpha, s.alphaDotFiltered, s.theta,
        s.thetaDotFiltered, velocityTarget,
        swingUp.energyError(s), sequence.acceleration,
        tau, modelVoltage, motor.enabled ? static_cast<double>(motor.current_sp) : 0.0,
        voltage, sequence.thetaReference,
        static_cast<unsigned long>(samplePeriodSumUs / sampleCount),
        static_cast<unsigned long>(readPeriodSumUs / sampleCount),
        static_cast<unsigned long>(maxSamplePeriodUs),
        static_cast<unsigned long>(alphaGlitchCount));
    diagnosticsLength = (length > 0 && static_cast<size_t>(length) < sizeof(diagnostics))
        ? static_cast<size_t>(length) : 0;
    diagnosticsSent = 0;
    maxSamplePeriodUs = 0;
    samplePeriodSumUs = 0;
    readPeriodSumUs = 0;
    sampleCount = 0;
  }
  const int serialSpace = Serial.availableForWrite();
  if (serialSpace > 0 && diagnosticsSent < diagnosticsLength) {
    const size_t count = std::min(diagnosticsLength - diagnosticsSent,
        std::min(static_cast<size_t>(serialSpace), static_cast<size_t>(32)));
    diagnosticsSent += Serial.write(
        reinterpret_cast<const uint8_t*>(diagnostics + diagnosticsSent), count);
  }

  // user communication
  command.run();
}
