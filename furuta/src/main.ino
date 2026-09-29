// Swing-up a energia del pendolo di Furuta.
// Riferimenti: README §7 (legge di swing-up) e §8 (da thetaDDot a coppia/V).
//
// Catena di controllo implementata nel loop():
//   1) errE -> thetaDDot_cmd        EnergySwingUpController::compute()
//   2) thetaDDot_cmd -> tau         FurutaDynamics::torqueForArmAcceleration()
//   3) tau -> i -> V                percorso A (README §8): i = tau/kt,
//                                   V = R*i + ke*thetaDot con ke = kt (SI)
//   4) V -> motore                  SimpleFOC in MotionControlType::torque
//
// FUNZIONI DA IMPLEMENTARE NELLA LIBRERIA furuta (segnate [LIB-TODO] nel codice):
//   - Motor::currentForTorque(double tau)             [motor.hpp]  i = tau/kt
//   - Motor::voltageForCurrent(double i, double w)    [motor.hpp]  V = R*i + kt*w
//   - ActuatorLimits::phaseResistance [ohm]           [parameters.hpp] (oggi assente)
//   - limiti maxTorque/maxCurrent (oggi vuoti)        [parameters.hpp]
//   - esposizione di dynamics.hpp/energy_swing_up.hpp da furuta.hpp (da valutare:
//     dynamics.hpp tira dentro Eigen)
// thetaDDot -> tau e' gia' in libreria (dynamics.hpp), esposta solo a questo
// sketch: la conversione usata qui e' quella, non una formula duplicata.

#include <furuta.hpp>
// dynamics.hpp usa Eigen (ArduinoEigen): incluso esplicitamente per non
// appesantire furuta.hpp per gli sketch che non ne hanno bisogno.
#include <furuta/dynamics.hpp>
#include <furuta/energy_swing_up.hpp>
#include <SimpleFOC.h>

// ===========================================================================
// Configurazione
// ===========================================================================

static constexpr uint32_t PERIOD_US = 1000;
static constexpr float FS_HZ = 1.06e6f / PERIOD_US;
static constexpr float    FC_HZ      = 20.0f;      // taglio passa-basso
static constexpr uint16_t HOMING_SAMPLES = 200;    // campioni consecutivi di quiete sullo zero (debounce)
static constexpr double   RAD_PER_COUNT  = 2.0 * M_PI / 4096.0;

// Offset meccanico del pendolo: alpha = 0 con il pendolo appeso (E = 0).
// Lo zero dell'AS5600 del pendolo e' ruotato di 180 gradi (da verificare:
// pendolo appeso fermo -> ErrE ~= E* = 2*m*g*l ~= 0.0279 J; se ErrE ~= 0
// l'offset e' sbagliato di pi).
static constexpr double ALPHA_OFFSET = M_PI;

// Segno di alpha nella convenzione del modello. La terna del modello e' fissata
// dal verso positivo del motore (theta = theta_FOC); il verso positivo di alpha
// dipende dall'orientamento della cerniera e dal montaggio dell'AS5600 del
// pendolo. Con lo swing-up attivo: se il motore smorza invece di pompare
// (ErrE cresce invece di calare), invertire questo segno.
static constexpr double swingDirection = -1.0;

// Profilo di homing (non usato dallo swing-up: theta non entra nella legge,
// serve solo thetaDot). Lasciato per compatibilita' con furuta.ino.
static constexpr float NOMINAL_P_ANGLE = 20.0f;
static constexpr float NOMINAL_VELOCITY_LIMIT = 50.0f * _2PI / 60.0f;
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

// Swing-up (README §7.2-7.3): k decide quanto in alto arriva il pendolo
// (piu' k = piu' vicino alla verticale), maxArmAcceleration e' il tetto di
// accelerazione del braccio e ne decide la violenza/rapidita'.
// TARGET_ENERGY_FRACTION < 1 ferma il target sotto E* (separatrice): il pendolo
// sale gradualmente in 3-4 s fino a ~35-40 gradi dalla verticale e ricade,
// senza mai passare in rotazione a 360 gradi.
// Sweep simulato host (RK4 1 ms): k=500, maxA=15, f=0.93 -> ~3.0-3.4 s,
// |thetaDot| <= ~3.6 rad/s, E_finale ~0.92 E*.
static constexpr double SWING_UP_GAIN = 500.0;
static constexpr double MAX_ARM_ACCELERATION = 15.0;
static constexpr double TARGET_ENERGY_FRACTION = 0.90;

// Resistenza di fase [ohm] per V = R*i + ke*thetaDot (README §8).
// [LIB-TODO] dovrebbe stare in ActuatorLimits (parameters.hpp); valore di
// partenza provvisorio, DA IDENTIFICARE sul motore reale.
static constexpr double PHASE_RESISTANCE = 1.38;

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
    furuta::VelocityFilterMode::Kalman, FC_HZ, FS_HZ};

MagneticSensorI2C armSensor = MagneticSensorI2C(AS5600_I2C);       // theta (bus Wire)
MagneticSensorI2C pendulumSensor = MagneticSensorI2C(AS5600_I2C);  // alpha (bus Wire1)
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C);
Commander command = Commander(Serial);

// ===========================================================================
// Conversione thetaDDot -> tau -> i -> V  (README §8, percorso A)
//
// 1) thetaDDot -> tau: inversa esatta del modello con cerniera passiva
//    (tau2 = 0), GIA' in libreria:
//      FurutaDynamics::torqueForArmAcceleration(alpha, thetaDot, alphaDot, thetaDDot)
//
// I passi 2 e 3 non esistono ancora nella libreria: vivono qui e sono marcati
// [LIB-TODO] con la firma che dovranno avere una volta portati in motor.hpp.
// ===========================================================================

// tau [N*m] -> corrente i [A]
static double torqueToCurrent(double tau)
{
    // [LIB-TODO] Motor::currentForTorque(tau)
    return tau / motorModel.kt();
}

// i [A] + thetaDot [rad/s] -> tensione q [V]: V = R*i + ke*thetaDot,
// con ke = kt in unita' SI (V*s/rad).
static double currentToVoltage(double current, double thetaDot)
{
    // [LIB-TODO] Motor::voltageForCurrent(current, thetaDot)
    return PHASE_RESISTANCE * current + motorModel.kt() * thetaDot;
}

// ===========================================================================
// Setup
// ===========================================================================

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Sensori: braccio sul bus Wire, pendolo sul bus Wire1 (stesso indirizzo I2C).
  Wire.begin(ARM_SDA, ARM_SCL);
  armSensor.init();
  Wire1.begin(PENDULUM_SDA, PENDULUM_SCL);
  pendulumSensor.init(&Wire1);

  // Il motore usa l'AS5600 del braccio per la commutazione FOC.
  motor.linkSensor(&armSensor);

  // driver config
  // power supply voltage [V]: alimentatore da banco a 24 V (ActuatorLimits::voltage)
  driver.voltage_power_supply = actuatorParams.voltage;
  driver.init();
  motor.linkDriver(&driver);

  // controllo di coppia in tensione: il target di move() e' Vq (README §8).
  // Limite = 24 V: il percorso A comanda V = R*i + ke*thetaDot, non la tensione
  // del bus, quindi il limite del bus serve solo a saturare il comando.
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = actuatorParams.voltage;

  motor.useMonitoring(Serial);

  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // Aggancia lo stimatore alla lettura corrente: il primo campione non
  // genera uno spike di velocita'.
  s.alpha = swingDirection * furuta::referenceAngle(pendulumSensor.getSensorAngle(), ALPHA_OFFSET);
  alphaEstimator.reset(s.alpha);
}

// ===========================================================================
// Loop
// ===========================================================================

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  motor.loopFOC();

  // Pendolo: alpha = 0 appeso + stima filtrata di alphaDot.
  s.alpha = swingDirection * furuta::referenceAngle(pendulumSensor.getSensorAngle(), ALPHA_OFFSET);
  s.alphaDotFiltered = alphaEstimator.update(s.alpha);

  // Braccio: thetaDot dal sensore FOC (sensor_direction gia' applicato da
  // SimpleFOC), quindi nello stesso verso positivo della coppia comandata.
  // motor.shaft_velocity e' aggiornato da move() al giro precedente (~1 ms).
  s.thetaDotFiltered = motor.shaft_velocity;

  // --- catena di controllo -------------------------------------------------
  // 1) legge di swing-up: errE -> thetaDDot_cmd (saturata su maxArmAcceleration)
  const furuta::controlOutput cmd = swingUp.compute(s);

  // 2) thetaDDot_cmd -> tau: inversa del modello, cerniera passiva tau2 = 0
  const double tau = dynamics.torqueForArmAcceleration(
      s.alpha, s.thetaDotFiltered, s.alphaDotFiltered, cmd.armAcceleration);

  // 3) tau -> i -> V (percorso A: i = tau/kt, V = R*i + ke*thetaDot)
  const double current = torqueToCurrent(tau);
  const double voltage = currentToVoltage(current, s.thetaDotFiltered);

  // 4) comando al motore (SimpleFOC satura su voltage_limit)
  motor.move(voltage);

  // --- diagnostica seriale (CSV: separatore ",") ---------------------------
  static unsigned long last_print = 0;
  if (millis() - last_print >= 100) {
    last_print = millis();
    Serial.print(F("Alpha: "));
    Serial.print(s.alpha, 4);
    Serial.print(F(","));
    Serial.print(F("AlphaDot stimata: "));
    Serial.print(s.alphaDotFiltered, 4);
    Serial.print(F(","));
    Serial.print(F("ThetaDot: "));
    Serial.print(s.thetaDotFiltered, 4);
    Serial.print(F(","));
    Serial.print(F("ErrE: "));
    Serial.print(swingUp.energyError(s), 6);
    Serial.print(F(","));
    Serial.print(F("ThetaDDot: "));
    Serial.print(cmd.armAcceleration, 3);
    Serial.print(F(","));
    Serial.print(F("Tau: "));
    Serial.print(tau, 5);
    Serial.print(F(","));
    Serial.print(F("I: "));
    Serial.print(current, 3);
    Serial.print(F(","));
    Serial.print(F("V: "));
    Serial.print(voltage, 3);
    Serial.print(F(","));
    Serial.print(F("Omega value Jxx: "));
    Serial.print(sqrt(arm.pendulumEnPot / arm.J_xx), 4);
    Serial.print(F(","));
    Serial.print(F("T value Jxx: "));
    Serial.print(2* M_PI * (1 / sqrt(arm.pendulumEnPot / arm.J_xx)), 4);
    Serial.print(F(","));
    Serial.print(F("Omega value Jyy: "));
    Serial.print(sqrt(arm.pendulumEnPot / arm.J_yy), 4);
    Serial.print(F(","));
    Serial.print(F("T value Jyy: "));
    Serial.println(2* M_PI * (1 / sqrt(arm.pendulumEnPot / arm.J_yy)), 4);
  }

  // user communication
  command.run();
}
