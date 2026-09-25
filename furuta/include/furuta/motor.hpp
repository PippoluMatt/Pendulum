#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "furuta/math_utils.hpp"
#include "furuta/parameters.hpp"

namespace furuta {

// Logica: deriva kV e kt dai dati.
class Motor
{
public:
    explicit Motor(const ActuatorLimits& params) noexcept
        : params_{params} {}

    double kV() const noexcept
    {
        return (params_.mode == 0)
            ? 1.0 / (kt() * RTRPS)
            : (params_.maxRPM / params_.voltage); 
    }

    double kt() const noexcept
    {
        return (params_.mode == 0)
            ? (params_.nPower / (params_.nRPM * RTRPS)) / 3.30
            : 1.0 / (kV()* RTRPS);
    }

    int mode() const noexcept { return params_.mode; }
    void setMode(int mode) noexcept { params_.mode = mode; }

private:
    ActuatorLimits params_;
    const double RTRPS {rpmToRadPerSec()};
};

} // namespace furuta
#endif
