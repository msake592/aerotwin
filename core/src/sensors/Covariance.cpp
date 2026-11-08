#include "Covariance.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace aerotwin::detail {
namespace {

// Keep the geometric mean in mantissa/exponent form so a subnormal
// denominator is never rounded before division.
double normalizedEntry(double value, double variance_a, double variance_b) {
    int exponent_a, exponent_b, exponent_value;
    const double a = std::frexp(variance_a, &exponent_a);
    const double b = std::frexp(variance_b, &exponent_b);
    const double fraction = std::frexp(value, &exponent_value);
    const int sum = exponent_a + exponent_b;
    const int half = sum / 2;
    const double denominator = std::sqrt(std::scalbn(a * b, sum - 2 * half));
    return std::scalbn(fraction / denominator, exponent_value - half);
}

} // namespace

std::optional<Eigen::Matrix3d> validatedCovariance(const Eigen::Matrix3d& input,
                                                  Eigen::Matrix3d* sampling_factor) {
    if (!input.allFinite() || (input.diagonal().array() < 0.0).any()) {
        return std::nullopt;
    }

    constexpr double tolerance = 1e-12;
    Eigen::Matrix3d symmetric = input;
    Eigen::Matrix3d correlation = Eigen::Matrix3d::Identity();
    for (int row = 1; row < 3; ++row) {
        for (int column = 0; column < row; ++column) {
            const double lower = input(row, column);
            const double upper = input(column, row);
            const double variance_a = input(row, row);
            const double variance_b = input(column, column);
            if (variance_a == 0.0 || variance_b == 0.0) {
                if (lower != 0.0 || upper != 0.0) {
                    return std::nullopt;
                }
                continue;
            }

            const double a = normalizedEntry(lower, variance_a, variance_b);
            const double b = normalizedEntry(upper, variance_a, variance_b);
            if (!std::isfinite(a) || !std::isfinite(b)) {
                return std::nullopt;
            }
            const double pair_scale = std::max({1.0, std::abs(a), std::abs(b)});
            if (std::abs(a / pair_scale - b / pair_scale) > tolerance) {
                return std::nullopt;
            }

            const double mean = std::midpoint(lower, upper);
            const double normalized = normalizedEntry(mean, variance_a, variance_b);
            if (!std::isfinite(normalized) || std::abs(normalized) > 1.0 + tolerance) {
                return std::nullopt;
            }
            symmetric(row, column) = symmetric(column, row) = mean;
            correlation(row, column) = correlation(column, row) = normalized;
        }
    }
    // A zero-variance axis is isolated. A unit placeholder on that axis
    // leaves the PSD decision on the positive-variance block unchanged.
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
        correlation, sampling_factor ? Eigen::ComputeEigenvectors : Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
        solver.eigenvalues().minCoeff() < -tolerance) {
        return std::nullopt;
    }
    if (sampling_factor) {
        const Eigen::Vector3d scales = symmetric.diagonal().cwiseSqrt();
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
