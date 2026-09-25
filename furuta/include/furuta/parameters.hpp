#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

namespace furuta {

// Dati puri: nessuna logica, trivially copyable (48 byte).
struct MechanicalParameters
{   
    // dimensions come from https://en.wikipedia.org/wiki/Furuta_pendulum
    const double pendulumMass{16.30};       // m2
    const double pendulumToCoM{46.35};      // l2
    const double pendulumLenght{94.4};      // L2

    const double pinwheelMass{96.57};       // m1
    const double pinwheelToCoM{7.02};       // l1
    const double pendulumToCenter{77.82};   // L1

    const double gravity{-9.81};
    
};

struct ActuatorLimits
{
    double maxRPM{3850.0};
    int voltage{24};
    int poles{8};
    double nPower{50.0};
    double nRPM{3000.0};

    double maxTorque{};
    double maxArmVelocity{};
    double maxArmAngle{};
    double minArmAngle{};
    double maxCurrent{};

    
    int mode{0};
};

struct  ControlOutput
{
    ;
};


} // namespace furuta
#endif
