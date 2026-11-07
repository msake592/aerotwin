# ADR 0008: Explicit checked body/world transformations

Status: Accepted (M1.4 merged in PR #7)

## Context

M1.3 supplies validated quaternion rotations. ENU/FLU consumers need clear point
and vector mappings without confusing rotation direction or translating vectors.

## Decision

Provide four functions in `<aerotwin/math/Transformations.hpp>`:
`bodyToWorldVector`, `worldToBodyVector`, `bodyToWorldPoint`, and `worldToBodyPoint`.
All take `q_world_from_body`; points additionally take the body origin in world
metres. Reuse `rotateVector` and `inverseRotation`, returning `std::optional<Vector3>`.
Follow the [coordinate contract](../architecture/coordinate-frames.md).

## Rationale and trade-offs

Explicit names and arguments fit existing Eigen aliases and avoid a new pose or
state abstraction before consumers exist. Vectors have no translation argument.
Unit/frame correctness remains the caller's responsibility. Revalidation in the
rotation primitives preserves one quaternion policy at a small accepted cost.
Check translation/subtraction results and reject nonfinite intermediates instead
of adding fused arithmetic or extended precision. Standard double rounding and
underflow remain documented limitations; finite results need not be exact.

## Consequences

Tests cover frame axes, inverse order, translation, round trips, invalid inputs,
and numerical edge cases. No dependencies are added. PX4 NED/FRD conversions are
documented as changes of both attitude bases, but adapters remain deferred.
Transform storage, composition, moving-frame kinematics, clocks, and states remain
outside this step.
