#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include "velocity_filters.hpp"
#include "furuta/sensor_data.hpp"
#include <algorithm>

namespace furuta
{
    inline static double rpmToRadPerSec() {return (2.0 * M_PI) / 60.0;}
    
    // Aggiorna le stime di velocita' dai due AS5600 (braccio e pendolo).
    // Un'istanza per sensore: ognuna mantiene il proprio stato del filtro.
    inline void updateSensors(SensorData& sensor,
                              AS5600VelocityEstimator& thetaEstimator,
                              AS5600VelocityEstimator& alphaEstimator) noexcept
    {
        sensor.thetaDotFiltered = thetaEstimator.update(sensor.theta);
        sensor.alphaDotFiltered = alphaEstimator.update(sensor.alpha);
    }

    inline double clamp(double x, double minValue, double maxValue)
    // Limita il valore x a rimanere nell'intervallo [minValue, maxValue]
    {
        return std::clamp(x, minValue, maxValue);
    }

    inline bool homing(const SensorData& sensor)
    // Homing: true se il pendolo e' fermo sulla posizione di riferimento
    // (alpha = 0). La tolleranza e' legata alla risoluzione dell'AS5600
    // (1 LSB = 2*pi/4096 rad ~ 1.53 mrad): l'epsilon di macchina (~2.2e-16 rad)
    // non sarebbe mai raggiungibile.
    // Serve anche il criterio di quiete: alpha passa per lo zero anche a
    // velocita' massima. Per un homing robusto il chiamante dovrebbe comunque
    // richiedere la condizione per piu' campioni consecutivi.
    {
        constexpr double oneLsb{2.0 * M_PI / 4096.0};
        constexpr double angleTolerance{2.0 * oneLsb};  // ~3.1 mrad
        constexpr double velocityTolerance{0.5};        // rad/s, sopra il rumore filtrato

        return std::abs(sensor.theta) < angleTolerance
            && std::abs(sensor.theta) < velocityTolerance;
    }

} // furuta namespace

#endif