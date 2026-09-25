#include "furuta/motor.hpp"
#include "furuta/math_utils.hpp"

#include <chrono>
#include <iostream>

int main()
{
    furuta::SensorData sensors {};
    double t {0.0};
    while (t<=5.0)
    {   
        sensors = furuta::getAngles(t);
        furuta::updateSensors(sensors, 0.01);
        std::cout << "Theta: " << sensors.theta << " ThetaDot: " << sensors.thetaDotFiltered
        << " Alpha: " << sensors.alpha << " AlphaDot: " <<  sensors.alphaDotFiltered
        << " ThetaPrev: " << sensors.thetaPrev << " AlphaPrev: " << sensors.alphaPrev << "\n\n";
        ++t;
    };
    

    furuta::MechanicalParameters parameters{};
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
