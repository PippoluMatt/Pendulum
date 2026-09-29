// Calcolo offline: nessuna soluzione della Riccati nel loop della board.
#include <furuta/parameters.hpp>
#include <furuta/lqr.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iomanip>
#include <iostream>

int main()
{
    const furuta::PinwheelParameters pinwheel;
    const furuta::ArmParameters arm;
    const furuta::FrictionParameters friction;
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    A(0, 2) = A(1, 3) = 1.0;
    A(3, 1) = arm.pendulumEnPot / arm.J_yy;
    A(3, 3) = -friction.pendulumViscous / arm.J_yy;
    const Eigen::Vector4d B(0.0, 0.0, 1.0,
        arm.pendulumMass * pinwheel.pendulumToCenter * arm.pendulumToCoM / arm.J_yy);
    const Eigen::Matrix4d Q = Eigen::Vector4d(1.0, 100.0, 1.0, 1.0).asDiagonal();
    constexpr double R = 0.1;

    // Sottospazio stabile dell'Hamiltoniana: P = U21 * U11^-1.
    Eigen::Matrix<double, 8, 8> H;
    H << A, -B * B.transpose() / R, -Q, -A.transpose();
    Eigen::ComplexEigenSolver<Eigen::Matrix<double, 8, 8>> solver(H);
    if (solver.info() != Eigen::Success) return 1;
    Eigen::Matrix<std::complex<double>, 8, 4> U;
    int count = 0;
    for (int i = 0; i < 8; ++i) {
        if (solver.eigenvalues()[i].real() < 0.0) {
            if (count == 4) return 1;
            U.col(count++) = solver.eigenvectors().col(i);
        }
    }
    if (count != 4) return 1;
    const Eigen::Matrix4cd complexP = U.bottomRows<4>() * U.topRows<4>().inverse();
    const Eigen::Matrix4d P = complexP.real();
    const Eigen::RowVector4d K = B.transpose() * P / R;
    const double residual = (A.transpose() * P + P * A
        - P * B * B.transpose() * P / R + Q).norm();
    const auto poles = (A - B * K).eval().eigenvalues().eval();
    const Eigen::RowVector4d firmwareK(furuta::LqrController::thetaGain,
        furuta::LqrController::alphaGain, furuta::LqrController::thetaDotGain,
        furuta::LqrController::alphaDotGain);
    std::cout << std::setprecision(12) << "K: " << K << "\nPoli: "
              << poles.transpose() << "\nResiduo CARE: " << residual << '\n';
    const bool valid = complexP.imag().norm() < 1e-7
        && (P - P.transpose()).norm() < 1e-7 && residual < 1e-7
        && poles.real().maxCoeff() < 0.0 && (K - firmwareK).norm() < 1e-6;
    if (!valid) std::cerr << "Riccati non valida o guadagni firmware da aggiornare\n";
    return valid ? 0 : 1;
}
