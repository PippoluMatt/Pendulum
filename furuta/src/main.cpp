#include "furuta/motor.hpp"
#include "furuta/math_utils.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

namespace
{
// Simulatore di letture AS5600 per il test su host: sulla board va
// sostituito con il driver I2C.
furuta::SensorData getAngles(double t)
{
    furuta::SensorData sensor{};
    sensor.theta = 0.3 * std::sin(0.5 * t);
    sensor.alpha = 0.8 * std::sin(2.0 * t);
    return sensor;
}
} // namespace

int main()
{
   

    furuta::ActuatorLimits parameters{};
    furuta::Motor motor {parameters};


    std::cout << "The mode value is: " << motor.mode() << "\n";
    std::cout << "The kt value is: " << motor.kt() << "\n";
    std::cout << "The kV value is: " << motor.kV() << "\n";

    motor.setMode(1);
    std::cout << "The mode value is: " << motor.mode() << "\n";
    std::cout << "After setMode(1), kt is: " << motor.kt() << "\n";
    std::cout << "The kV value is: " << motor.kV() << "\n";
    return 0;
}
