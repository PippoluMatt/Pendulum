#ifndef SWING_SEQUENCE_HPP
#define SWING_SEQUENCE_HPP

namespace furuta {

#include <furuta/math_utils.hpp>

// Sequenza del solo sketch, condivisa con i test host.
class SwingSequence {
public:
    enum class Phase { Homing, Settling, Waiting, Swing, Returning };
    static constexpr double armLimit = 2.0 * M_PI;
    static constexpr double maxVelocity = 5.0;
    static constexpr double maxAcceleration = 20.0;

    Phase phase{Phase::Homing};
    double velocityTarget{0.0};
    double acceleration{0.0};

    void update(const furuta::SensorData& s, double energyAcceleration, double dt)
    {
        acceleration = 0.0;
        const bool centered = furuta::atRest(s.theta, s.thetaDotFiltered);
        switch (phase) {
        case Phase::Homing:
        case Phase::Returning:
            quietTime = centered ? quietTime + dt : 0.0;
            if (quietTime >= 0.2) {
                phase = Phase::Settling;
                quietTime = 0.0;
            }
            break;
        case Phase::Settling:
            // L'allineamento e l'homing possono far oscillare il pendolo.
            quietTime = centered && std::abs(s.alpha) < 5.0 * M_PI / 180.0
                && std::abs(s.alphaDotFiltered) < 0.15 ? quietTime + dt : 0.0;
            if (quietTime >= 1.0) {
                restAlpha = s.alpha;
                phase = Phase::Waiting;
                quietTime = 0.0;
            }
            break;
        case Phase::Waiting:
            // Una spinta di 20 gradi supera la soglia; rumore e offset no.
            quietTime = std::abs(s.alpha - restAlpha) >= 10.0 * M_PI / 180.0
                ? quietTime + dt : 0.0;
            if (quietTime >= 0.02) {
                phase = Phase::Swing;
                quietTime = 0.0;
            }
            break;
        case Phase::Swing: {
            // Riserva 0.5 rad e una distanza di frenata stimata a 5 rad/s^2.
            // Usa la posizione continua e la velocita' MISURATA, non il target.
            const double outwardSpeed = s.theta * s.thetaDotFiltered > 0.0
                ? std::abs(s.thetaDotFiltered) : 0.0;
            if (std::abs(s.theta) + outwardSpeed * outwardSpeed / 10.0
                    >= armLimit - 0.5) {
                phase = Phase::Returning;
                quietTime = 0.0;
                break;
            }
            // Il richiamo al centro rimuove la componente continua che prima
            // si accumulava integrando solo l'accelerazione energetica.
            const double centeredAcceleration = furuta::clamp(
                energyAcceleration - 1.0 * s.theta - 0.5 * velocityTarget,
                -maxAcceleration, maxAcceleration);
            const double previous = velocityTarget;
            velocityTarget = furuta::clamp(previous + centeredAcceleration * dt,
                                            -maxVelocity, maxVelocity);
            acceleration = (velocityTarget - previous) / dt;
            break;
        }
        }
        if (phase != Phase::Swing) velocityTarget = 0.0;
    }

    const char* name() const
    {
        switch (phase) {
        case Phase::Homing: return "HOMING";
        case Phase::Settling: return "SETTLING";
        case Phase::Waiting: return "WAIT_PUSH";
        case Phase::Swing: return "SWING";
        case Phase::Returning: return "RETURN_LIMIT";
        }
        return "";
    }

private:
    double quietTime{0.0};
    double restAlpha{0.0};
};
} // furuta namespace
#endif