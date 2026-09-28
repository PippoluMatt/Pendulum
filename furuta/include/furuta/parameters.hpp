#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

namespace furuta {

inline constexpr double gravity{9.81};
inline constexpr double mu{0.00125};

// Dati puri: nessuna logica, trivially copyable (48 byte).
struct PinwheelParameters
{   
    const double pinwheelMass{0.09657};       // m1
    const double pinwheelToCoM{0.00702};       // l1
    const double pendulumToCenter{0.07782};   // L1  
};

struct ActuatorLimits
{
    double maxRPM{3850.0};
    int voltage{24};
    int poles{8};
    double nPower{50.0};
    double nRPM{3000.0};

    double maxTorque{};
    double maxCurrent{};
    
    int mode{0};
};

struct ArmParameters
{
    // dimensions come from https://en.wikipedia.org/wiki/Furuta_pendulum
    const double pendulumMass{0.0210};       // m2
    const double pendulumToCoM{0.04635};      // l2
    const double pendulumLenght{0.0944};      // L2
    
    const double pendulumEnPot{pendulumMass * gravity * pendulumToCoM};
    const double J_xx {3.962E-4};
    const double J_yy {2.68E-4};
};

struct  ControlOutput
{
    ;
};


} // namespace furuta
#endif
