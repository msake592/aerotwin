# Coordinate frame conventions

Status: M1.4 DONE, merged in [PR #7](https://github.com/msake592/aerotwin/pull/7).
M1.3 merged in
[PR #6](https://github.com/msake592/aerotwin/pull/6), with all four
[CI jobs passing](https://github.com/msake592/aerotwin/actions/runs/42124882965).
See [mathematical contracts](math-contracts.md) for types and units.

| Frame | Axes | Intended use |
| --- | --- | --- |
| World ENU | X east, Y north, Z up | Canonical world frame for core-facing simulation contracts |
| World NED | X north, Y east, Z down | PX4-facing aerospace integration |
| Body FLU | X forward, Y left, Z up | Core-facing body convention |
| Body FRD | X forward, Y right, Z down | PX4-facing body convention |
| Sensor-local | Declared per sensor | Measurement generation and interpretation |

All listed conventions are right-handed. ENU/NED world frames and FLU/FRD body frames are different concepts and must not be used interchangeably. Gazebo world alignment and geodetic origin must be configured explicitly; a simulator's world axes must not be assumed to mean east and north automatically.

Future adapters must explicitly convert between declared source and destination frames. Interfaces must document vector expression frame, pose reference frame, transform direction, sensor extrinsics, and covariance ordering. For interchange, serialize unit quaternions as `[w, x, y, z]` using the Hamilton convention. A body attitude actively maps body-frame vectors into world coordinates using `v_world = q * v_body * conjugate(q)`, with vectors embedded as pure quaternions. The inverse maps world vectors into body coordinates. Serialize components explicitly at library and adapter boundaries.

Use metres for position, metres per second for velocity, radians for angles, and radians per second for angular rates. Declare the expression frame and units for noise and covariance as well. Use [run-relative simulation time](../adr/0004-simulation-time.md) for observations and states. [M1.3 rotation primitives](quaternion-rotations.md) supply all rotation mathematics below.

## Checked body/world transformations

Include `<aerotwin/math/Transformations.hpp>` and link `aerotwin_core`.
All four functions are in `aerotwin`, take const references, leave inputs unchanged,
and return `std::optional<Vector3>`. Argument order is quaternion, then position
(for points only), then the value to transform. Both directions take the same
`q_world_from_body`: the active Hamilton rotation from body FLU to world ENU.
No fixed basis permutation is implied by ENU/FLU labels: at identity, forward
aligns east, left north, and up up. A positive 90-degree yaw maps forward north
and left west.

| Function | Formula | Arguments after quaternion |
| --- | --- | --- |
| `bodyToWorldVector` | `v_world = R(q) * v_body` | `vector_body` |
| `worldToBodyVector` | `v_body = R(q)^-1 * v_world` | `vector_world` |
| `bodyToWorldPoint` | `p_world = R(q) * p_body + position_world_m` | `position_world_m, point_body_m` |
| `worldToBodyPoint` | `p_body = R(q)^-1 * (p_world - position_world_m)` | `position_world_m, point_world_m` |

`position_world_m` is the body origin expressed in world ENU metres. Points are
in metres relative to the respective origin. Vectors retain their input units;
directions/displacements rotate without translation and need not have unit norm.
These operations express components at the same event time; they do not implement
moving-frame velocity derivatives. Callers own compatible frames, world origin,
units, event time, and truth/estimate provenance. Aliases provide no compile-time
frame or point checking. No transform object, clock, state model, or adapter is added.

```cpp
#include <aerotwin/math/Transformations.hpp>
#include <cmath>

const double s = std::sqrt(0.5);
const aerotwin::Quaternion q_world_from_body(s, 0, 0, s); // Hamilton WXYZ
const aerotwin::Vector3 position_world_m(10, 20, 30);
const auto point_world_m = aerotwin::bodyToWorldPoint(
    q_world_from_body, position_world_m, aerotwin::Vector3(2, 0, 0));
if (point_world_m) { // Approximately (10, 22, 30).
    const auto point_body_m = aerotwin::worldToBodyPoint(
        q_world_from_body, position_world_m, *point_world_m);
    // Check point_body_m before using it; approximately (2, 0, 0).
}
const auto direction_world = aerotwin::bodyToWorldVector(
    q_world_from_body, aerotwin::Vector3::UnitX()); // Approximately north (0, 1, 0).
// Check direction_world before using it. worldToBodyVector uses the same quaternion.
```

## Validation and numerical limits

All supplied points, positions, and vectors must be finite. Quaternions must pass
the existing default `isUnitQuaternion` norm tolerance (`1e-12`); rotation operations
remove accepted rounding drift per M1.3, without accepting arbitrary scaled inputs.
Zero/nonunit/nonfinite rotations are rejected. No silent identity or zero fallback
is used. Failure is `std::nullopt`, with no additional error classification.

Forward points first call the checked rotation, then add position componentwise.
Reverse points subtract position componentwise, check the displacement, then call
the checked inverse rotation. Nonfinite translation, subtraction, or rotation
results are rejected. In particular, an overflowed rotated point is rejected even
if a later translation could cancel it; an overflowed displacement is rejected
even if a later rotation could produce finite components. No fused or reordered
transform attempts to recover those cases. M1.3's internal overflow retry policy
is preserved; these wrappers validate the result returned by that primitive.

Double rounding, cancellation, and gradual underflow are unavoidable. Small
translations can round away beside large coordinates; subnormal products may
underflow to zero. A finite result does not prove exact accuracy, and round trips
cannot recover lost bits. Identity with zero translation preserves representable
mixed-magnitude/subnormal components under the default floating-point environment.
Tests use absolute `1e-12` for moderate metre-scale values and exact comparisons
where representability warrants them; this is not a universal physical tolerance.
See the [rotation numerical limits](quaternion-rotations.md). Fast-math and
flush-to-zero are unsupported. No extra precision, saturation, or epsilon-based
repair is introduced.

## PX4 boundary: NED/FRD

Future PX4 adapters must keep these conversions at the boundary:

- ENU to NED: `(x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)`.
  NED to ENU uses the same mapping.
- FLU to FRD: `(x_frd, y_frd, z_frd) = (x_flu, -y_flu, -z_flu)`.
  FRD to FLU uses the same mapping.

For coincident world origins define `C_ned_from_enu = [[0,1,0],[1,0,0],[0,0,-1]]`
and `C_flu_from_frd = diag(1,-1,-1)`. Both are proper rotations. An active attitude
must convert **both** its source and destination bases:
`R_ned_from_frd = C_ned_from_enu * R_enu_from_flu * C_flu_from_frd`.
The inverse relation is
`R_enu_from_flu = C_enu_from_ned * R_ned_from_frd * C_frd_from_flu`.
Equivalent Hamilton quaternion composition uses the same left/right order with
unit basis-change quaternions; merely shuffling or negating attitude coefficients
as if they were vector components is incorrect. Quaternion signs are equivalent;
serialize named WXYZ components and verify the external protocol's direction and
ordering. World positions also require declared origin/alignment; a basis mapping
alone supplies no geodetic translation. Covariance/extrinsic conversions need their
own boundary contracts. No PX4 conversion API or middleware adapter is implemented
in M1.4. See [ADR 0008](../adr/0008-coordinate-transformations.md).
