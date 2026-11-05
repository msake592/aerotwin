#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace aerotwin {

// Double-precision XYZ components. Units and expression frame belong to each API contract.
// Eigen default construction does not initialize coefficients; initialize explicitly.
using Vector3 = Eigen::Vector3d;

// Hamilton quaternion. Constructor order is (w, x, y, z), but coeffs() is XYZW.
// Body attitude maps body FLU to world ENU. Orientation values must be finite and unit length.
using Quaternion = Eigen::Quaterniond;

// Absolute tolerance on quaternion norm (dimensionless); no implicit normalization.
inline constexpr double kQuaternionNormTolerance = 1e-12;

[[nodiscard]] inline bool isFinite(const Vector3& vector) noexcept {
    return vector.allFinite();
}

// Invalid tolerances, zero quaternions, NaN, and infinity are rejected.
[[nodiscard]] inline bool isUnitQuaternion(
    const Quaternion& quaternion, double tolerance = kQuaternionNormTolerance) noexcept {
    if (!std::isfinite(tolerance) || tolerance < 0.0 || !quaternion.coeffs().allFinite()) {
        return false;
    }
    const double norm = quaternion.coeffs().stableNorm();
    return norm > 0.0 && std::isfinite(norm) && std::abs(norm - 1.0) <= tolerance;
}

} // namespace aerotwin
