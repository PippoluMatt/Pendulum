#ifndef LQR_HPP
#define LQR_HPP

#include <furuta/math_utils.hpp>

namespace furuta {

// LQR continuo con ingresso thetaDDot, non coppia. Stato:
// [theta - thetaReference, wrap(alpha - pi), thetaDot, alphaDot].
// Guadagni per parameters.hpp, Q=diag(1,100,1,1), R=0.1.
// Rigenerazione e verifica della Riccati: tools/lqr_design.cpp.
class LqrController {
public:
    static constexpr double thetaGain = -3.162277660168;
    static constexpr double alphaGain = 372.549511782;
    static constexpr double thetaDotGain = -4.76104548034;
    static constexpr double alphaDotGain = 25.4076741677;
    static constexpr double entryAngle = 2.3;
    static constexpr double exitAngle = 2.1;
    static constexpr double entrySpeed = 15.0;
    static constexpr double exitSpeed = 18.0;

    static double uprightError(double alpha)
    {
        return std::remainder(alpha - M_PI, 2.0 * M_PI);
    }

    double compute(const SensorData& s, double thetaReference) const
    {
        return -(thetaGain * (s.theta - thetaReference)
               + alphaGain * uprightError(s.alpha)
               + thetaDotGain * s.thetaDotFiltered
               + alphaDotGain * s.alphaDotFiltered);
    }

    bool canCapture(const SensorData& s, double maxAcceleration) const
    {
        // Il limite sul comando seleziona una fascia di velocita' compatibile
        // con l'angolo: a 2.3 rad quasi fermo il pendolo viene rifiutato.
        return std::abs(uprightError(s.alpha)) <= M_PI - entryAngle
            && std::abs(s.alphaDotFiltered) <= entrySpeed
            && std::abs(compute(s, s.theta)) <= maxAcceleration;
    }

    bool lostBalance(const SensorData& s) const
    {
        return std::abs(uprightError(s.alpha)) > M_PI - exitAngle
            || std::abs(s.alphaDotFiltered) > exitSpeed;
    }
};
} // furuta namespace
#endif
