#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

namespace furuta {

inline constexpr double gravity{9.81};
inline constexpr double mu{0.00125};

// Dati puri: nessuna logica, trivially copyable (48 byte).
// Geometria dal CAD (dimensions.txt): J1 = braccio.
struct PinwheelParameters
{   
    const double pinwheelMass{0.096567};      // m1, massa J1
    const double pinwheelToCoM{0.0070202};    // l1, asse motore -> CoM braccio
    const double pendulumToCenter{0.079073};  // L1, asse motore -> cerniera pendolo
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

// Geometria dal CAD (dimensions.txt): J2 = pendolo, origine componente = cerniera.
struct ArmParameters
{
    const double pendulumMass{0.021332};      // m2, massa J2
    const double pendulumToCoM{0.066611};     // l2, cerniera -> CoM
    const double pendulumLenght{0.0859};      // L2, cerniera -> punta (12 mm di accoppiamento sopra l'asse)
    
    const double pendulumEnPot{pendulumMass * gravity * pendulumToCoM};
    const double J_xx {9.9256E-7};  // J2xx: asse longitudinale del pendolo (frame CoM)
    const double J_yy {1.0513E-4};  // hat J2: asse cerniera (J2zz + trasporto m2*l2^2)
};

struct  ControlOutput
{
    ;
};


} // namespace furuta
#endif
