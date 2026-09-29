#ifndef ENERGY_SWING_UP
#define ENERGY_SWING_UP

#include "furuta/parameters.hpp"
#include "furuta/math_utils.hpp"
#include "furuta/sensor_data.hpp"

#include <cmath>

namespace furuta{

// Parametri della legge di swing-up (README §7.2-7.3). I default sono punti
// di partenza da tarare: `gain` decide la rapidita' del pompaggio, mentre
// `maxArmAcceleration` riflette coppia disponibile e inerzia vista dal motore.
// `targetEnergyFraction` < 1 pone il target sotto la verticale (E* e' la
// separatrice); non garantisce l'assenza di sovraelongazioni o rotazioni reali.
struct EnergySwingUpConfig
{
    double gain{500.0};                // k [rad/s^2 / (J * rad/s)]
    double maxArmAcceleration{100.0};  // |thetaDDot_cmd| massima [rad/s^2]
    double targetEnergyFraction{1.0};  // E_target = frazione di E* (1.0 = verticale)
};

struct controlOutput
{
    double armAcceleration{0.0};  // thetaDDot_cmd [rad/s^2]
};

class EnergySwingUpController
{
public:
    explicit EnergySwingUpController(const ArmParameters& arm,
                                     const EnergySwingUpConfig& config = {}) noexcept
        : arm_{arm}, config_{config} {}

    // Inerzia del pendolo rispetto all'asse di rotazione (cerniera),
    // gia' comprensiva del termine di trasporto.
    double pendulumInertia() const noexcept
    {
        return arm_.J_yy;
    }

    // E = 1/2 J alphadot^2 + m g l (1 - cos(alpha))
    double energy(const SensorData& sensor) const noexcept
    {
        const double kinetic = 0.5 * pendulumInertia()
                             * sensor.alphaDotFiltered * sensor.alphaDotFiltered;
        return kinetic
             + arm_.pendulumEnPot * (1.0 - std::cos(sensor.alpha));
    }

    // Pendolo fermo in verticale (alpha = pi): E* = 2 m g l, scalata da
    // targetEnergyFraction (< 1 = target sotto la verticale).
    double targetEnergy() const noexcept
    {
        return config_.targetEnergyFraction * 2.0 * arm_.pendulumEnPot;
    }

    // errE = E* - E
    double energyError(const SensorData& sensor) const noexcept
    {
        return targetEnergy() - energy(sensor);
    }

    // Legge di swing-up (README §7.2):
    //   thetaDDot_cmd = -k * errE * alphaDot * cos(alpha)
    // Il segno negativo e' fisico: dall'equazione del pendolo
    //   Edot = -m2*L1*l2*cos(alpha)*thetaDDot*alphaDot
    //          + (1/2)*J2*sin(2*alpha)*thetaDot^2*alphaDot
    // quindi per avere Edot >= 0 con errE > 0 serve thetaDDot opposta a
    // alphaDot*cos(alpha).
    controlOutput compute(const SensorData& sensor) const noexcept
    {
        const double acceleration = -config_.gain * energyError(sensor)
                                  * sensor.alphaDotFiltered
                                  * std::cos(sensor.alpha);
        controlOutput output{};
        output.armAcceleration = clamp(acceleration,
                                       -config_.maxArmAcceleration,
                                       +config_.maxArmAcceleration);
        return output;
    }

private:
    ArmParameters arm_;
    EnergySwingUpConfig config_;
};

} // furuta namespace
#endif
