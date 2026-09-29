#ifndef SWING_SEQUENCE_HPP
#define SWING_SEQUENCE_HPP

#include <furuta/math_utils.hpp>
#include <furuta/lqr.hpp>

namespace furuta {

// Sequenza del solo sketch, condivisa con i test host.
class SwingSequence {
public:
    enum class Phase { Homing, Settling, Waiting, Swing, Balance, Returning };
    static constexpr double armLimit = 2.0 * M_PI;
    static constexpr double maxVelocity = 5.0;
    static constexpr double maxAcceleration = 20.0;

    Phase phase{Phase::Homing};
    double velocityTarget{0.0};
    double acceleration{0.0};
    double thetaReference{0.0}; // acquisito una sola volta per ogni cattura

    bool controlsVelocity() const
    {
        return phase == Phase::Swing || phase == Phase::Balance;
    }

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
            // Il braccio e' libero (driver disabilitato dallo sketch durante
            // attesa e assestamento): senza forza di richiamo non si puo'
            // pretendere |theta| ~ 0. Conta solo la quiete del pendolo; il
            // richiamo al centro avviene durante lo swing.
            quietTime = std::abs(s.alpha) < 5.0 * M_PI / 180.0
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
        case Phase::Swing:
        case Phase::Balance: {
            // Riserva 0.5 rad e una distanza di frenata stimata a 5 rad/s^2.
            // Usa la posizione continua e la velocita' MISURATA, non il target.
            const double distance = std::abs(s.theta);
            const double outwardSpeed = s.theta * s.thetaDotFiltered > 0.0
                ? std::abs(s.thetaDotFiltered) : 0.0;
            // Il termine di velocita' vale solo nella fascia esterna (5 rad dal
            // limite): a theta ~ 0 un picco della stima (quantizzazione x jitter
            // del loop) predirebbe una frenata inesistente e farebbe scattare
            // il rientro a vuoto (osservato sul banco: RETURN_LIMIT con
            // theta ~ -0.1). Nella fascia esterna la predizione resta attiva.
            // In BALANCE la predizione vale solo se il pendolo e' uscito dal
            // range LQR: dentro il range e' l'LQR a frenare e un picco di
            // velocita' dell'arm interrompeva recuperi riusciti (osservato sul
            // banco: RETURN_LIMIT con alpha 3.07, e_alpha -0.07). Il limite
            // pieno resta sempre attivo come ultima riserva.
            const bool predictive = phase == Phase::Swing || lqr.lostBalance(s);
            if (distance >= armLimit - 0.5
                    || (predictive && distance >= armLimit - 5.0
                        && distance + outwardSpeed * outwardSpeed / 10.0
                            >= armLimit - 0.5)) {
                phase = Phase::Returning;
                quietTime = 0.0;
                break;
            }
            if (phase == Phase::Swing && lqr.canCapture(s, maxAcceleration)) {
                thetaReference = s.theta;
                phase = Phase::Balance;
            } else if (phase == Phase::Balance && lqr.lostBalance(s)) {
                phase = Phase::Swing;
            }
            // Il richiamo al centro rimuove la componente continua che prima
            // si accumulava integrando solo l'accelerazione energetica.
            // In Balance il riferimento e' invece la posizione di cattura.
            const double requestedAcceleration = furuta::clamp(
                phase == Phase::Balance ? lqr.compute(s, thetaReference)
                    : energyAcceleration - 1.0 * s.theta - 0.5 * velocityTarget,
                -maxAcceleration, maxAcceleration);
            const double previous = velocityTarget;
            velocityTarget = furuta::clamp(previous + requestedAcceleration * dt,
                                            -maxVelocity, maxVelocity);
            acceleration = (velocityTarget - previous) / dt;
            break;
        }
        }
        if (!controlsVelocity()) velocityTarget = 0.0;
    }

    const char* name() const
    {
        switch (phase) {
        case Phase::Homing: return "HOMING";
        case Phase::Settling: return "SETTLING";
        case Phase::Waiting: return "WAIT_PUSH";
        case Phase::Swing: return "SWING";
        case Phase::Balance: return "BALANCE";
        case Phase::Returning: return "RETURN_LIMIT";
        }
        return "";
    }

private:
    LqrController lqr;
    double quietTime{0.0};
    double restAlpha{0.0};
};
} // furuta namespace
#endif
