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

    // Percorso A (README §8): tau [N*m] -> corrente i [A].
    // Nessuna saturazione su maxCurrent (oggi non identificato): il limite va
    // applicato dal chiamante, altrimenti l'inversa del modello viene alterata
    // in silenzio.
    double currentForTorque(double tau) const noexcept
    {
        return tau / kt();
    }

    // Percorso A (README §8): i [A] + thetaDot [rad/s] -> tensione q [V],
    // V = R*i + ke*thetaDot con ke = kt in unita' SI (V*s/rad).
    double voltageForCurrent(double current, double thetaDot) const noexcept
    {
        return params_.phaseResistance * current + kt() * thetaDot;
    }

private:
    ActuatorLimits params_;
    const double RTRPS {rpmToRadPerSec()};
};

} // namespace furuta
#endif
