#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include "furuta/parameters.hpp"

#ifdef ARDUINO
#include <EigenDense.h>  // ArduinoEigen: header del pacchetto installato nelle libraries
#else
#include <Eigen/Dense>
#endif
#include <cmath>

namespace furuta {

// q = [theta, alpha]^T con i suoi vettori e matrici 2x2. Alias Eigen per
// tenere leggibile il codice; lo stesso tipo si userà per lo stato di RK4.
using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

// Dinamica del Furuta pendulum in forma lumped (README §4.3-4.4):
//   M(q) qdd + C(q, qd) qd + G(q) + D qd = B tau,  B = [1, 0]^T
// con la cerniera passiva: la seconda equazione ha forze generalizzate nulle.
class FurutaDynamics
{
public:
    FurutaDynamics(const PinwheelParameters& pinwheel,
                   const ArmParameters& arm,
                   const FrictionParameters& friction = {}) noexcept
        : J0_{pinwheel.pinwheelInertia
              + pinwheel.pinwheelMass * pinwheel.pinwheelToCoM * pinwheel.pinwheelToCoM
              + arm.pendulumMass * pinwheel.pendulumToCenter * pinwheel.pendulumToCenter},
          J2_{arm.J_yy},
          coupling_{arm.pendulumMass * pinwheel.pendulumToCenter * arm.pendulumToCoM},
          gravityTorque_{arm.pendulumEnPot},
          b1_{friction.armViscous},
          b2_{friction.pendulumViscous} {}

    double armInertia() const noexcept { return J0_; }         // hat J0
    double pendulumInertia() const noexcept { return J2_; }    // hat J2
    double coupling() const noexcept { return coupling_; }     // m2*L1*l2
    double gravityTorque() const noexcept { return gravityTorque_; }  // m2*g*l2

    Mat2 massMatrix(double alpha) const noexcept
    {
        const double sinAlpha = std::sin(alpha);
        const double coupling = coupling_ * std::cos(alpha);

        Mat2 M;
        M << J0_ + J2_ * sinAlpha * sinAlpha, coupling,
             coupling,                       J2_;
        return M;
    }

    // C(q, qd) qd + D qd (h di README §4.4).
    Vec2 biasForces(double alpha, double thetaDot, double alphaDot) const noexcept
    {
        const double sinAlpha = std::sin(alpha);
        const double sin2Alpha = std::sin(2.0 * alpha);

        Vec2 h;
        h << -coupling_ * sinAlpha * alphaDot * alphaDot
                + J2_ * sin2Alpha * thetaDot * alphaDot
                + b1_ * thetaDot,
             -0.5 * J2_ * sin2Alpha * thetaDot * thetaDot
                + b2_ * alphaDot;
        return h;
    }

    Vec2 gravityVector(double alpha) const noexcept
    {
        Vec2 g;
        g << 0.0, gravityTorque_ * std::sin(alpha);
        return g;
    }

    // Risolve il modello per qdd dati q, qd e la coppia tau:
    //   qdd = M(q)^-1 (B tau - C(q, qd) qd - G(q) - D qd)
    Vec2 solveAcceleration(double alpha, double thetaDot, double alphaDot,
                           double tau) const noexcept
    {
        Vec2 rhs = -biasForces(alpha, thetaDot, alphaDot) - gravityVector(alpha);
        rhs.x() += tau;  // B tau, con B = [1, 0]^T

        // M(q) e' simmetrica definita positiva: LDLT e' diretta e senza pivot.
        return massMatrix(alpha).ldlt().solve(rhs);
    }

    // Coppia che impone thetaDDot sull'asse del motore rispettando la cerniera
    // passiva (tau2 = 0): la seconda equazione fornisce alphaDDot di reazione,
    // la prima la coppia.
    double torqueForArmAcceleration(double alpha, double thetaDot, double alphaDot,
                                    double thetaDDot) const noexcept
    {
        const Mat2 M = massMatrix(alpha);
        const Vec2 b = biasForces(alpha, thetaDot, alphaDot);
        const Vec2 g = gravityVector(alpha);

        const double alphaDDot = -(M(1, 0) * thetaDDot + b(1) + g(1)) / M(1, 1);
        return M(0, 0) * thetaDDot + M(0, 1) * alphaDDot + b(0) + g(0);
    }

private:
    double J0_;
    double J2_;
    double coupling_;
    double gravityTorque_;
    double b1_;
    double b2_;
};

} // furuta namespace
#endif
