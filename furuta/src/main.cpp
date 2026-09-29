#include "furuta/dynamics.hpp"
#include "furuta/energy_swing_up.hpp"
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

    // Smoke test del collegamento errE -> thetaDDot_cmd -> tau: la legge di
    // swing-up genera l'accelerazione dell'asse, il modello dinamico la
    // traduce in coppia rispettando la cerniera passiva.
    furuta::PinwheelParameters pinwheel{};
    furuta::ArmParameters arm{};
    furuta::FrictionParameters friction{};
    furuta::FurutaDynamics dynamics{pinwheel, arm, friction};
    furuta::EnergySwingUpController swingUp{arm};

    furuta::SensorData sensor{};
    sensor.alpha = M_PI / 3.0;
    sensor.alphaDotFiltered = 1.0;

    const furuta::controlOutput command = swingUp.compute(sensor);
    const double tau = dynamics.torqueForArmAcceleration(
        sensor.alpha, sensor.thetaDotFiltered, sensor.alphaDotFiltered,
        command.armAcceleration);

    std::cout << "Energy error: " << swingUp.energyError(sensor) << " J\n";
    std::cout << "thetaDDot_cmd: " << command.armAcceleration << " rad/s^2\n";
    std::cout << "tau_cmd: " << tau << " N*m\n";

    // Il modello risolto con tau_cmd deve riprodurre thetaDDot_cmd.
    const furuta::Vec2 qdd = dynamics.solveAcceleration(
        sensor.alpha, sensor.thetaDotFiltered, sensor.alphaDotFiltered, tau);
    std::cout << "thetaDDot from model: " << qdd.x() << " rad/s^2\n";
    return 0;
}
