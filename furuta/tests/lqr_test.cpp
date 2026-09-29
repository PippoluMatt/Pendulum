#include "lqr_simulation.hpp"
#include <Eigen/Eigenvalues>
#include <cstdlib>
#include <iostream>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void checkLinearization()
{
    const furuta::FurutaDynamics dynamics({}, {});
    const auto derivative = [&](const Eigen::Vector4d& x, double u) -> Eigen::Vector4d {
        const double tau = dynamics.torqueForArmAcceleration(x[1], x[2], x[3], u);
        const auto dd = dynamics.solveAcceleration(x[1], x[2], x[3], tau);
        return {x[2], x[3], dd[0], dd[1]};
    };
    const Eigen::Vector4d upright(0.0, M_PI, 0.0, 0.0);
    constexpr double h = 1e-5;
    Eigen::Matrix4d A;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector4d perturbation = Eigen::Vector4d::Zero();
        perturbation[i] = h;
        A.col(i) = (derivative(upright + perturbation, 0.0)
            - derivative(upright - perturbation, 0.0)) / (2.0 * h);
    }
    const Eigen::Vector4d B = (derivative(upright, h) - derivative(upright, -h)) / (2.0 * h);
    const furuta::LqrController lqr;
    Eigen::RowVector4d K;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector4d x = upright;
        x[i] += h;
        K[i] = -lqr.compute({x[0], x[1], x[2], x[3]}, 0.0) / h;
    }
    require((A - B * K).eval().eigenvalues().real().maxCoeff() < -1.0,
        "LQR stabilizza la linearizzazione numerica della dinamica reale");
    for (double side : {-1.0, 1.0}) {
        const furuta::SensorData s{0.7, side * M_PI, 0.0, 0.0};
        require(std::abs(lqr.compute(s, 0.7)) < 1e-10, "equilibrio a entrambi i wrap");
    }
    require(std::abs(lqr.compute({0., M_PI + .01, 0., .2}, 0.)
        - lqr.compute({0., -M_PI + .01, 0., .2}, 0.)) < 1e-10,
        "comando continuo attraverso il wrap del sensore");
}

void checkVelocityRestLock()
{
    constexpr float dt = 0.00047f;
    constexpr double lsb = 2.0 * M_PI / 4096.0;
    constexpr float zone = 15.0f * static_cast<float>(M_PI) / 180.0f;
    furuta::AS5600VelocityEstimator hanging(furuta::VelocityFilterMode::Kalman, 20.f,
        1000.f, 1.0e-6f, 1.0e-2f, 0.5f, 0.1667f, 0.05f, zone);
    hanging.reset(0.0f);
    // Dither di 1 LSB a pendolo appeso: dopo 50 ms l'uscita deve essere 0.
    for (int i = 0; i < 400; ++i) hanging.update(i % 2 ? lsb : -lsb, dt);
    require(hanging.update(0.0, dt) == 0.0f, "a riposo appeso la velocita' e' zero");
    // Un movimento vero sblocca e torna a stimare.
    double angle = 0.0;
    float velocity = 0.0f;
    for (int i = 0; i < 200; ++i) {
        angle += 0.5 * dt;
        velocity = hanging.update(angle, dt);
    }
    require(velocity > 0.3f, "il movimento reale sblocca il rest-lock");

    // Vicino alla verticale la zona di riposo non deve applicarsi.
    furuta::AS5600VelocityEstimator upright(furuta::VelocityFilterMode::Kalman, 20.f,
        1000.f, 1.0e-6f, 1.0e-2f, 0.5f, 0.1667f, 0.05f, zone);
    upright.reset(M_PI);
    bool estimated = false;
    for (int i = 0; i < 200; ++i)
        estimated |= upright.update(M_PI + (i % 2 ? lsb : -lsb), dt) != 0.0f;
    require(estimated, "vicino alla verticale il filtro resta a banda piena");
}

void checkSequence()
{
    using Phase = furuta::SwingSequence::Phase;
    furuta::SwingSequence startup;
    furuta::SensorData resting{};
    for (int i = 0; i < 1300; ++i) startup.update(resting, 0., .001);
    require(startup.phase == Phase::Waiting, "homing e quiete prima dell'attesa");
    resting.alpha = .35;
    for (int i = 0; i < 21; ++i) startup.update(resting, 0., .001);
    require(startup.phase == Phase::Swing, "spinta avvia lo swing");

    for (double sign : {-1.0, 1.0}) {
        furuta::SwingSequence sequence;
        sequence.phase = Phase::Swing;
        furuta::SensorData s{sign * .7, sign * 2.31, 0., 0.};
        sequence.update(s, 10., .001);
        require(sequence.phase == Phase::Swing, "apice a 2.3 rad senza velocita' rifiutato");
        s.alphaDotFiltered = -sign * 12.;
        sequence.update(s, 10., .001);
        require(sequence.phase == Phase::Swing, "moto lontano dalla verticale rifiutato");
        s.alphaDotFiltered = sign * 16.;
        sequence.update(s, 10., .001);
        require(sequence.phase == Phase::Swing, "velocita' eccessiva rifiutata");
        s.alpha = sign * 2.29;
        s.alphaDotFiltered = sign * 12.3;
        sequence.update(s, 10., .001);
        require(sequence.phase == Phase::Swing, "angolo sotto soglia rifiutato");
        s.alpha = sign * 2.31;
        const double oldTarget = sequence.velocityTarget;
        sequence.update(s, 10., .001);
        require(sequence.phase == Phase::Balance, "cattura su entrambi i lati");
        require(sequence.thetaReference == s.theta, "riferimento acquisito all'ingresso");
        require(std::abs(sequence.velocityTarget - oldTarget) <= .02000001,
            "nessun salto nel riferimento di velocita' alla cattura");
        const double reference = sequence.thetaReference;
        s.theta += sign * .1;
        s.alpha = sign * 2.2;
        sequence.update(s, 10., .001);
        require(sequence.phase == Phase::Balance && sequence.thetaReference == reference,
            "isteresi angolare e riferimento fisso durante bilanciamento");
        s.alpha = sign * 2.0;
        sequence.update(s, 10., .001);
        require(sequence.phase == Phase::Swing, "perdita angolare riprende swing senza attesa");
        s.alpha = sign * 2.31;
        sequence.update(s, 10., .001);
        require(sequence.phase == Phase::Balance && sequence.thetaReference == s.theta,
            "ricattura acquisisce il nuovo riferimento");
        s.alphaDotFiltered = sign * 19.;
        sequence.update(s, 10., .001);
        require(sequence.phase == Phase::Swing, "eccesso di velocita' riprende swing");

        // La protezione del braccio precede anche una cattura ammissibile.
        // Il limite pieno vale in entrambe le fasi; il termine predittivo in
        // BALANCE scatta solo col pendolo fuori dal range LQR.
        for (Phase phase : {Phase::Swing, Phase::Balance}) {
            for (double theta : {2.0 * M_PI - .49, 2.0 * M_PI + .1}) {
                sequence.phase = phase;
                s = {sign * theta, sign * M_PI, sign * 3., 0.};
                sequence.update(s, 20., .001);
                require(sequence.phase == Phase::Returning && sequence.velocityTarget == 0.
                    && sequence.acceleration == 0., "limite theta prioritario e continuo");
            }
        }
        sequence.phase = Phase::Swing;
        s = {sign * 5.0, sign * M_PI, sign * 3., 0.};
        sequence.update(s, 20., .001);
        require(sequence.phase == Phase::Returning, "in SWING la frenata stimata protegge l'arm");
        sequence.phase = Phase::Balance;
        sequence.thetaReference = 0.;
        s = {sign * 5.0, sign * M_PI, sign * 3., 0.};
        sequence.update(s, 20., .001);
        require(sequence.phase == Phase::Balance,
            "in BALANCE, pendolo in range: un picco dell'arm non interrompe l'LQR");
        sequence.phase = Phase::Balance;
        sequence.thetaReference = 0.;
        s = {sign * 5.0, sign * 2.0, sign * 3., 0.};
        sequence.update(s, 20., .001);
        require(sequence.phase == Phase::Returning,
            "in BALANCE, pendolo fuori range: la protezione predittiva riprende");
        sequence.phase = Phase::Balance;
        sequence.thetaReference = 0.;
        sequence.velocityTarget = sign * 4.99;
        s = {0., sign * 2.3, 0., 0.};
        sequence.update(s, 0., .01);
        require(std::abs(sequence.velocityTarget - sign * 5.) < 1e-10,
            "riferimento di velocita' saturato anche in LQR");
        require(std::abs(sequence.acceleration - sign * 1.) < 1e-10,
            "feed-forward usa accelerazione effettiva dopo saturazione");
    }
}

void checkSimulations()
{
    for (double dt : {.00075, .001, .002}) {
        for (double sign : {-1.0, 1.0}) {
            // Equilibrio locale e cattura dinamica dalla soglia richiesta.
            for (bool capture : {false, true}) {
                const auto r = simulate({sign * .7, sign * (capture ? 2.3 : M_PI - .03),
                    0., capture ? sign * 12. : 0.}, 10., dt);
                std::cout << "dt=" << dt << " lato=" << sign << " cattura=" << capture
                    << " ingresso=" << r.captureTime << " mantenimento=" << r.balancedTime
                    << " errore=" << furuta::LqrController::uprightError(r.state[1])
                    << " erroreMaxFinale=" << r.finalAngleError
                    << " erroreThetaMaxFinale=" << r.finalReferenceError
                    << " thetaMax=" << r.peakTheta << " Vmax=" << r.peakVoltage << '\n';
                require(r.state.allFinite() && r.captureTime >= 0. && r.captureTime < .1,
                    "cattura nel modello non lineare con PI, quantizzazione e ritardo");
                require(r.balancedTime > 9.8 && r.losses == 0 && !r.returned,
                    "mantenimento continuo, nessuna commutazione ripetuta");
                // A 2 ms il PI e la quantizzazione producono un ciclo limite
                // (con maxA=25 circa 2.1 gradi / 0.18 rad); sotto 1 ms valgono
                // i criteri stretti.
                require(r.finalAngleError < (dt < .002 ? .01 : 2.1 * M_PI / 180.)
                    && r.finalReferenceError < (dt < .002 ? .02 : .18),
                    "errore limitato nell'ultimo secondo di mantenimento");
                require(r.peakTheta < furuta::SwingSequence::armLimit && r.peakVoltage <= 2.000001,
                    "limiti fisici nel modello simulato");
            }
            const auto boundary = simulate({sign * 5., sign * M_PI, sign * 3., 0.}, 2., dt);
            require(boundary.returned && boundary.peakTheta < furuta::SwingSequence::armLimit,
                "rientro simulato prima del limite con velocita' uscente");
        }
    }
    // Sensibilita': i guadagni nominali restano locali e stabili senza attrito
    // viscoso alla cerniera. Non equivale a un'identificazione hardware.
    const auto lowFriction = simulate({.4, M_PI - .03, 0., 0.}, 10., .001, .95, {.0006, 0.});
    require(lowFriction.balancedTime > 9.8 && lowFriction.losses == 0
        && std::abs(furuta::LqrController::uprightError(lowFriction.state[1])) < .01,
        "stabilizzazione locale anche con attrito del pendolo nullo");

    // Ciclo completo dal basso: con target 1.10 la pompa supera la
    // separatrice e l'LQR cattura. b2 e' un'ipotesi (non identificato sul
    // banco): con b2 >= 1e-4 l'autorita' di pompaggio attuale non basta.
    const auto fromRest = simulate({0., 20. * M_PI / 180., 0., 0.}, 25., .001,
        1.10, {.0006, 2e-5});
    std::cout << "Swing completo: cattura=" << fromRest.captureTime
              << " s, bilanciato=" << fromRest.balancedTime
              << " s, thetaMax=" << fromRest.peakTheta << '\n';
    require(fromRest.captureTime >= 0.0 && fromRest.captureTime < 6.0
        && fromRest.balancedTime > 15.0 && fromRest.losses == 0 && !fromRest.returned
        && fromRest.peakTheta < furuta::SwingSequence::armLimit
        && fromRest.peakVoltage <= 2.000001,
        "swing completo dal basso: cattura e mantenimento");
}
} // namespace

int main()
{
    checkLinearization();
    checkVelocityRestLock();
    checkSequence();
    checkSimulations();
    std::cout << "PASS LQR, commutazione e simulazioni non lineari\n";
}
