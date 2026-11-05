#include <aerotwin/math/Rotations.hpp>

namespace aerotwin {
namespace {

std::optional<Quaternion> checkedRotation(const Quaternion& rotation) {
    if (!isUnitQuaternion(rotation)) {
        return std::nullopt;
    }
    return normalizeQuaternion(rotation);
}

} // namespace

std::optional<Quaternion> normalizeQuaternion(const Quaternion& quaternion) {
    if (!quaternion.coeffs().allFinite()) {
        return std::nullopt;
    }
    const double scale = quaternion.coeffs().cwiseAbs().maxCoeff();
    if (scale == 0.0) {
        return std::nullopt;
    }
    // Scaling first handles finite coefficients whose full norm overflows, and
    // tiny nonzero inputs without squaring them into zero.
    Quaternion result;
    result.coeffs() = quaternion.coeffs() / scale;
    result.coeffs() /= result.coeffs().norm();
    if (!isUnitQuaternion(result)) {
        return std::nullopt;
    }
    return result;
}

std::optional<Quaternion> composeRotations(const Quaternion& after, const Quaternion& before) {
    const auto unit_after = checkedRotation(after);
    const auto unit_before = checkedRotation(before);
    if (!unit_after || !unit_before) {
        return std::nullopt;
    }
    return normalizeQuaternion(*unit_after * *unit_before);
}

std::optional<Quaternion> inverseRotation(const Quaternion& rotation) {
    const auto unit = checkedRotation(rotation);
    if (!unit) {
        return std::nullopt;
    }
    return unit->conjugate();
}

std::optional<Vector3> rotateVector(const Quaternion& rotation, const Vector3& vector) {
    const auto unit = checkedRotation(rotation);
    if (!unit || !isFinite(vector)) {
        return std::nullopt;
    }
    const Eigen::Matrix3d matrix = unit->toRotationMatrix();
    // Do not scale finite rows: doing so could erase small, representable
    // components merely because an unrelated vector component is very large.
    Vector3 result = matrix * vector;
    for (int row = 0; row < 3; ++row) {
        if (!std::isfinite(result[row])) {
            // Retry only an overflowed row with bounded intermediates. This
            // also handles partial-sum overflow followed by cancellation.
            // A nonfinite row with finite inputs implies a nonzero vector.
            const double scale = vector.cwiseAbs().maxCoeff();
            result[row] = matrix.row(row).dot(vector / scale) * scale;
        }
    }
    if (!isFinite(result)) {
        return std::nullopt;
    }
    return result;
}

} // namespace aerotwin
