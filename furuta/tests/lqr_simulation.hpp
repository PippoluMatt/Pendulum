#ifndef LQR_SIMULATION_HPP
#define LQR_SIMULATION_HPP

// Modello di verifica host: RK4, motore resistivo, PI/LPF e ritardo di un
// campione come nello sketch. Non identifica attriti o corrente del banco.
#include <furuta/dynamics.hpp>
#include <furuta/energy_swing_up.hpp>
#include <furuta/motor.hpp>
#include <furuta/swing_sequence.hpp>

struct SimulationResult {
    Eigen::Vector4d state;
    double captureTime{-1.0};
    double balancedTime{0.0};
    double peakTheta{0.0};
    double peakVoltage{0.0};
    double reference{0.0};
    double finalAngleError{0.0}; // massimo nell'ultimo secondo
    double finalReferenceError{0.0};
    double minUprightError{M_PI}; // minima distanza dal top raggiunta
    double maxEnergy{0.0};
    int losses{0};
    bool returned{false};
};

inline SimulationResult simulate(Eigen::Vector4d x, double duration, double dt,
                                 double energyFraction = 0.95,
                                 furuta::FrictionParameters friction = {},
                                 double gain = 500.0, double swingMaxA = 20.0)
{
    const furuta::ArmParameters arm;
    const furuta::PinwheelParameters pinwheel;
    const furuta::ActuatorLimits limits;
    const furuta::Motor motor(limits);
    const furuta::FurutaDynamics dynamics(pinwheel, arm, friction);
    const furuta::EnergySwingUpController swing(arm, {gain, swingMaxA, energyFraction});
    furuta::SwingSequence sequence;
    sequence.phase = furuta::SwingSequence::Phase::Swing;
    sequence.velocityTarget = x[2];
    furuta::AS5600VelocityEstimator estimator(furuta::VelocityFilterMode::AlphaBeta, 20., 1000.);
    estimator.reset(std::remainder(x[1], 2.0 * M_PI));
    // Inizializza la stima con la velocita' iniziale prima della cattura.
    for (int i = 100; i >= 0; --i)
        estimator.update(std::remainder(x[1] - i * dt * x[3], 2.0 * M_PI), dt);
    double measuredVelocity = x[2], previousTheta = x[0];
    double integral = 0.0, previousError = 0.0, previousPi = 0.0, appliedVoltage = 0.0;
    SimulationResult result{};
    constexpr double lsb = 2.0 * M_PI / 4096.0;
    for (double time = 0.0; time < duration; time += dt) {
        const double theta = std::round(x[0] / lsb) * lsb;
        furuta::SensorData s{theta, std::remainder(std::round(x[1] / lsb) * lsb, 2.0 * M_PI),
            measuredVelocity, 0.0};
        s.alphaDotFiltered = estimator.update(s.alpha, dt);
        result.minUprightError = std::min(result.minUprightError,
            std::abs(furuta::LqrController::uprightError(s.alpha)));
        result.maxEnergy = std::max(result.maxEnergy, swing.energy(s));
        const auto previousPhase = sequence.phase;
        const bool wasVelocityControl = sequence.controlsVelocity();
        sequence.update(s, swing.compute(s).armAcceleration, dt);
        if (sequence.phase == furuta::SwingSequence::Phase::Balance) {
            if (result.captureTime < 0.0) result.captureTime = time;
            result.balancedTime += dt;
            result.reference = sequence.thetaReference;
        } else {
            if (previousPhase == furuta::SwingSequence::Phase::Balance) ++result.losses;
            result.balancedTime = 0.0;
        }
        if (sequence.phase == furuta::SwingSequence::Phase::Returning) result.returned = true;
        if (previousPhase != sequence.phase && !(wasVelocityControl && sequence.controlsVelocity()))
            integral = previousError = previousPi = 0.0;
        const double ff = sequence.controlsVelocity() ? furuta::clamp(motor.voltageForCurrent(
            motor.currentForTorque(dynamics.torqueForArmAcceleration(s.alpha,
                s.thetaDotFiltered, s.alphaDotFiltered, sequence.acceleration)),
            s.thetaDotFiltered), -1.0, 1.0) : 0.0;
        measuredVelocity = (0.005 * measuredVelocity + theta - previousTheta) / (0.005 + dt);
        previousTheta = theta;
        const double target = sequence.controlsVelocity() ? sequence.velocityTarget
            : furuta::clamp(-8.0 * s.theta, -2.0, 2.0);
        const double error = target - measuredVelocity;
        const double piLimit = sequence.controlsVelocity() ? 1.0 : 2.0;
        integral = furuta::clamp(integral + 20.0 * dt * 0.5 * (error + previousError), -piLimit, piLimit);
        const double pi = furuta::clamp(furuta::clamp(0.2 * error + integral, -piLimit, piLimit),
            previousPi - 100.0 * dt, previousPi + 100.0 * dt);
        previousError = error;
        previousPi = pi;
        const bool disabled = sequence.phase == furuta::SwingSequence::Phase::Settling
            || sequence.phase == furuta::SwingSequence::Phase::Waiting;
        const double voltage = disabled ? 0.0 : ff + pi;
        result.peakVoltage = std::max(result.peakVoltage, std::abs(voltage));
        const auto derivative = [&](const Eigen::Vector4d& y) -> Eigen::Vector4d {
            const double tau = disabled ? 0.0 : motor.kt()
                * (appliedVoltage - motor.kt() * y[2]) / limits.phaseResistance;
            const auto dd = dynamics.solveAcceleration(y[1], y[2], y[3], tau);
            return {y[2], y[3], dd[0], dd[1]};
        };
        const int steps = static_cast<int>(std::ceil(dt / 0.0002));
        const double h = dt / steps;
        for (int i = 0; i < steps; ++i) {
            const auto k1 = derivative(x), k2 = derivative(x + h / 2.0 * k1);
            const auto k3 = derivative(x + h / 2.0 * k2), k4 = derivative(x + h * k3);
            x += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
            result.peakTheta = std::max(result.peakTheta, std::abs(x[0]));
        }
        appliedVoltage = voltage;
        if (time >= duration - 1.0) {
            result.finalAngleError = std::max(result.finalAngleError,
                std::abs(furuta::LqrController::uprightError(x[1])));
            result.finalReferenceError = std::max(result.finalReferenceError,
                std::abs(x[0] - result.reference));
        }
    }
    result.state = x;
    return result;
}
#endif
