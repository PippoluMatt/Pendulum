#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include "furuta/sensor_data.hpp"
#include <algorithm>
#include <cmath>
#include <limits>


namespace furuta
{
    inline static double rpmToRadPerSec() {return (2.0 * M_PI) / 60.0;}

    class AS5600VelocityEstimator{
    public:
        AS5600VelocityEstimator(float fc_hz, float fs_hz) : dt(1.0f / fs_hz)
        {
            float rc = 1.0f / 2.0 * static_cast<float>(M_PI) * fc_hz;
            beta = dt / (rc + dt);
        }
    private:
        const float dt;
        float beta{0.0};
    };



    SensorData getAngles(double t){
        SensorData sensor{};
        sensor.theta = 0.3 * std::sin(0.5 * t);
        sensor.alpha = 0.8 * std::sin(2.0 * t);
        return sensor;
    }

    void updateSensors(SensorData& angles_, double dt)
    {
        

        angles_.thetaPrev = angles_.theta;
        angles_.alphaPrev = angles_.alpha;
    
    } // Con Filtro Passa-Basso

    inline double clamp(double x, double minValue, double maxValue)
    // Limita il valore x a rimanere nell'intervallo [minValue, maxValue]
    {
        return std::clamp(x, minValue, maxValue);
    }

    inline bool homing(const SensorData& sensor)
    // Homing: true se alpha e' a 0.0 rad entro la precisione di macchina
    {
        constexpr double precision{std::numeric_limits<double>::epsilon()};
        const double error{std::abs(sensor.alpha - 0.0)};
        return error < precision;
    }

    void lowPassFilter(SensorData& sensordata, float beta, float dt){
        const double thetaDotRaw = (sensordata.theta - sensordata.thetaPrev) / dt;
        const double alphaDotRaw = (sensordata.alpha - sensordata.alphaPrev) / dt;
 
        sensordata.thetaDotFiltered = (beta * thetaDotRaw) + (1.0 - beta) * sensordata.thetaDotFiltered;
        sensordata.alphaDotFiltered = (beta * alphaDotRaw) + (1.0 - beta) * sensordata.alphaDotFiltered;
    }

} // furuta namespace

#endif