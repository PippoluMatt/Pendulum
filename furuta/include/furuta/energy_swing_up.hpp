#ifndef ENERGY_SWING_UP
#define ENERGY_SWING_UP

#include "furuta/parameters.hpp"
#include "furuta/math_utils.hpp"
#include "furuta/sensor_data.hpp"

#include <cmath>

namespace furuta{

class EnergySwingUpController
{
public:
    explicit EnergySwingUpController(const ArmParameters& arm) noexcept
        : arm_{arm} {}

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

    // Pendolo fermo in verticale (alpha = pi): E* = 2 m g l
    double targetEnergy() const noexcept
    {
        return 2.0 * arm_.pendulumEnPot;
    }

    // errE = E* - E
    double energyError(const SensorData& sensor) const noexcept
    {
        return targetEnergy() - energy(sensor);
    }

private:
    ArmParameters arm_;
};




struct EnergySwingUpConfig
{

};


struct controlOutput
{
    float velocity{0.0};
    };

} // furuta namespace
#endif