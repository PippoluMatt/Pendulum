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

    // J1zz, inerzia del braccio rispetto al CoM (asse verticale, dal CAD).
    // L'Izz all'origine del CAD (8.7901e-5) include gia' m1*l1^2: si usa il
    // valore al CoM per non contare due volte il termine di trasporto in J0.
    const double pinwheelInertia{8.3141106E-05};
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

// Attriti viscosi (README §4.3): b1 sull'asse del motore, b2 sulla cerniera.
// Non ancora identificati: default a zero, il modello resta completo.
struct FrictionParameters
{
    double armViscous{0.0};       // b1 [N*m*s/rad]
    double pendulumViscous{0.0};  // b2 [N*m*s/rad]
};

} // namespace furuta
#endif
