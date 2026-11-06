#include <aerotwin/math/Transformations.hpp>

#include <aerotwin/math/Rotations.hpp>

namespace aerotwin {

std::optional<Vector3> bodyToWorldVector(const Quaternion& q_world_from_body,
                                         const Vector3& vector_body) {
    return rotateVector(q_world_from_body, vector_body);
}

std::optional<Vector3> worldToBodyVector(const Quaternion& q_world_from_body,
                                         const Vector3& vector_world) {
    const auto inverse = inverseRotation(q_world_from_body);
    if (!inverse) {
        return std::nullopt;
    }
    return rotateVector(*inverse, vector_world);
}

std::optional<Vector3> bodyToWorldPoint(const Quaternion& q_world_from_body,
                                        const Vector3& position_world_m,
                                        const Vector3& point_body_m) {
    if (!isFinite(position_world_m)) {
        return std::nullopt;
    }
    const auto rotated = bodyToWorldVector(q_world_from_body, point_body_m);
    if (!rotated) {
        return std::nullopt;
    }
    const Vector3 result = *rotated + position_world_m;
    if (!isFinite(result)) {
        return std::nullopt;
    }
    return result;
}

std::optional<Vector3> worldToBodyPoint(const Quaternion& q_world_from_body,
                                        const Vector3& position_world_m,
                                        const Vector3& point_world_m) {
    if (!isFinite(position_world_m) || !isFinite(point_world_m)) {
        return std::nullopt;
    }
    const Vector3 displacement_world_m = point_world_m - position_world_m;
    if (!isFinite(displacement_world_m)) {
        return std::nullopt;
    }
    return worldToBodyVector(q_world_from_body, displacement_world_m);
}

} // namespace aerotwin
