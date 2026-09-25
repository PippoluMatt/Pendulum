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

    // Tolleranze del criterio di quiete, condivise con chi fa controlli
    // aggiuntivi sullo stesso asse (es. la velocita' stimata da SimpleFOC).
    inline constexpr double HOMING_ANGLE_TOLERANCE = 2.0 * (2.0 * M_PI / 4096.0);  // ~3.1 mrad
    inline constexpr double HOMING_VELOCITY_TOLERANCE = 0.4;                        // rad/s

    inline bool atRest(double angle, double angleDot)
    // Homing: true se l'asse e' fermo sulla posizione di riferimento
    // (angolo = 0). La tolleranza e' legata alla risoluzione dell'AS5600
    // (1 LSB = 2*pi/4096 rad ~ 1.53 mrad): l'epsilon di macchina (~2.2e-16 rad)
    // non sarebbe mai raggiungibile.
    // Serve anche il criterio di quiete: l'angolo passa per lo zero anche a
    // velocita' massima. Per un homing robusto il chiamante dovrebbe comunque
    // richiedere la condizione per piu' campioni consecutivi.
    {
        return std::abs(angle) < HOMING_ANGLE_TOLERANCE
            && std::abs(angleDot) < HOMING_VELOCITY_TOLERANCE;
    }

    inline double referenceAngle(double rawAngle, double offset)
    {
        return clamp(rawAngle - offset, -2.0 * M_PI, 2.0*M_PI);
    }

    inline double homeTarget(double sensorDirection, double offset)
    {
        return sensorDirection * offset;
    }

} // furuta namespace

#endif