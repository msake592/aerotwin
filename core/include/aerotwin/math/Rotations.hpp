#pragma once

#include <aerotwin/math/Types.hpp>

#include <optional>

namespace aerotwin {

// Explicitly normalize any finite, nonzero quaternion, preserving its sign.
// Returns nullopt on invalid input or an unrepresentable result.
[[nodiscard]] std::optional<Quaternion> normalizeQuaternion(const Quaternion& quaternion);

// Active Hamilton composition: result = after * before, applying before first.
// Both inputs must satisfy isUnitQuaternion() with its default tolerance.
[[nodiscard]] std::optional<Quaternion> composeRotations(const Quaternion& after,
                                                       const Quaternion& before);

// Inverse active rotation (conjugate of the unit quaternion).
// Input must satisfy isUnitQuaternion() with its default tolerance.
[[nodiscard]] std::optional<Quaternion> inverseRotation(const Quaternion& rotation);

// Active rotation v' = q * (0,v) * conjugate(q), without translation.
// Requires a unit quaternion within the default tolerance and a finite vector.
// Units are preserved; input/output frames are specified by the caller.
// Returns nullopt on invalid inputs or nonfinite output. Accepted norm drift is
// removed before rotation (also for composition/inverse); inputs are not mutated.
[[nodiscard]] std::optional<Vector3> rotateVector(const Quaternion& rotation, const Vector3& vector);

} // namespace aerotwin
