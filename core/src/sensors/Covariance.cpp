#include "Covariance.hpp"

#include "../math/CovarianceValidation.hpp"

namespace aerotwin::detail {

std::optional<Eigen::Matrix3d> validatedCovariance(const Eigen::Matrix3d& input,
                                                  Eigen::Matrix3d* sampling_factor) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    const auto symmetric = validateCovariance(
        input, solver, sampling_factor ? Eigen::ComputeEigenvectors : Eigen::EigenvaluesOnly);
    if (!symmetric) {
        return std::nullopt;
    }
    if (sampling_factor) {
        const Eigen::Vector3d scales = symmetric->diagonal().cwiseSqrt();
        const Eigen::Vector3d roots = solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
        const Eigen::Matrix3d factor =
            scales.asDiagonal() * (solver.eigenvectors() * roots.asDiagonal());
        if (!factor.allFinite()) {
            return std::nullopt;
        }
        *sampling_factor = factor;
    }
    return symmetric;
}

} // namespace aerotwin::detail
