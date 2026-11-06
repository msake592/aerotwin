#pragma once

#include <aerotwin/math/Types.hpp>

#include <optional>

namespace aerotwin {

// World: ENU (east, north, up); body: FLU (forward, left, up), both right-handed.
// q_world_from_body actively maps body components to world components (Hamilton).
// All operations require the default isUnitQuaternion tolerance and finite inputs;
// M1.3 removes accepted norm drift, but rejects invalid rotations without repair.
// Returns nullopt on invalid input or nonfinite transform-stage intermediates/results.
// M1.3 may recover internal rotation overflow with its scaled retry; rotation
// results still nonfinite after recovery are rejected.
// Inputs are not mutated. Callers own frame, origin, unit, and event-time consistency.

// Rotation only: v_world = R(q_world_from_body) * v_body. Vector units are preserved;
// directions need not be unit length. No translation applies to vectors/displacements.
[[nodiscard]] std::optional<Vector3> bodyToWorldVector(const Quaternion& q_world_from_body,
                                                       const Vector3& vector_body);

// Rotation only: v_body = R(q_world_from_body)^-1 * v_world.
[[nodiscard]] std::optional<Vector3> worldToBodyVector(const Quaternion& q_world_from_body,
                                                       const Vector3& vector_world);

// Points and position are in metres. position_world_m is the body origin in ENU.
// p_world = R(q_world_from_body) * p_body + position_world_m.
[[nodiscard]] std::optional<Vector3> bodyToWorldPoint(const Quaternion& q_world_from_body,
                                                      const Vector3& position_world_m,
                                                      const Vector3& point_body_m);

// p_body = R(q_world_from_body)^-1 * (p_world - position_world_m).
// Subtraction is checked before rotation; overflow is rejected even if a combined
// exact-arithmetic transform might be finite. Double underflow/rounding may lose
// small terms; no exact round-trip guarantee is made. See coordinate-frames.md.
[[nodiscard]] std::optional<Vector3> worldToBodyPoint(const Quaternion& q_world_from_body,
                                                      const Vector3& position_world_m,
                                                      const Vector3& point_world_m);

} // namespace aerotwin
