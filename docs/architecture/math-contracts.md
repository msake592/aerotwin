# Mathematical types and unit contracts

Status: M1.2 types and validation and M1.3 rotations are merged. M1.4 checked
[coordinate transformations](coordinate-frames.md) are implemented, pending review.

## Public types

Include `<aerotwin/math/Types.hpp>` and link `aerotwin_core`.
`aerotwin::Vector3` is `Eigen::Vector3d` (three double XYZ components).
`aerotwin::Quaternion` is `Eigen::Quaterniond`, using Hamilton multiplication
(`i*j = k`) and constructor order `(w, x, y, z)`. Interchange order is always
`[w, x, y, z]`: read/write named accessors explicitly. Eigen's `coeffs()` and raw
storage are **[x, y, z, w]** and must not be copied as the interchange format.
No serialization helper is implemented. Checked operations are provided separately
in `<aerotwin/math/Rotations.hpp>`. Raw Eigen operations remain accessible through
these aliases and bypass AeroTwin validation.

Aliases preserve Eigen interoperability without duplicate arithmetic or wrapper
ownership. They do not enforce units, frames, point semantics, or normalization at
compile time. Do not default-construct and then read either type: Eigen leaves
coefficients uninitialized. Use `Vector3::Zero()`, explicit components, or
`Quaternion::Identity()` as appropriate. Zero position means the declared origin;
identity attitude means aligned axes, not an unknown attitude.

## Units and semantics

| Quantity | Unit | Required context |
| --- | --- | --- |
| Position / point | meters (m) | Reference origin and expression frame |
| Displacement | meters (m) | Expression frame |
| Velocity | meters/second (m/s) | Relative to which frame and expressed in which frame |
| Duration | seconds (s) | Finite; positivity as required by consuming API |
| Angle | radians (rad) | Rotation axis and direction |
| Angular velocity | radians/second (rad/s) | Relative to which frame and expressed in which frame |
| Direction | dimensionless | Expression frame; unit length if required |
| Orientation quaternion | dimensionless | Source and destination frame; unit norm |

Seconds describe floating-point durations, not a new timestamp representation.
[ADR 0004](../adr/0004-simulation-time.md) still governs future timestamps; no clock
is implemented. Fields/parameters should carry semantic names such as
`position_world_m`, `displacement_body_m`, `velocity_world_mps`, and
`angular_velocity_body_radps`. State owners must distinguish truth from estimates;
these mathematical types carry neither provenance nor timestamps.

A point locates something relative to an origin. Point minus point yields a
displacement; point plus displacement yields a point. Displacements and directions
are vectors; translation never applies to them. Adding two points is not a
meaningful geometric operation even though the alias permits it. A velocity is a
vector but is not a unit direction. Normalization must be requested explicitly.

## Frames and attitude

World ENU is right-handed: X east, Y north, Z up. Body FLU is right-handed:
X forward, Y left, Z up. A world origin/alignment must be declared by the future
scenario; ENU alone supplies neither a geodetic origin nor a map location.
Positive rotations follow the right-hand rule. `q_world_from_body` actively maps
body components into world components:
`v_world = q_world_from_body * (0, v_body) * conjugate(q_world_from_body)`.
The inverse maps world to body. Both quaternion signs describe the same attitude;
no sign canonicalization is required. Basis-vector and rotation tests are covered
for rotation primitives in M1.3; M1.4 tests cover body/world point and vector mappings.
See [coordinate frames](coordinate-frames.md).

## Coordinate transformations

Include `<aerotwin/math/Transformations.hpp>`. The four checked functions
`bodyToWorldVector`, `worldToBodyVector`, `bodyToWorldPoint`, and
`worldToBodyPoint` return `std::optional<Vector3>` and use `q_world_from_body`
in both directions. See [coordinate frames](coordinate-frames.md) for signatures,
formulas, translation, failures, numerical limits, and PX4 boundary conventions.
No transform storage type or composition API is needed for these operations.
Moving-frame kinematics, adapters, and timestamp ownership remain future work.

## Invalid values and invariants

`isFinite(Vector3)` rejects NaN and either infinity in any component.
`isUnitQuaternion(q, tolerance)` requires finite coefficients, nonzero norm, and
`abs(norm - 1) <= tolerance`; the default absolute norm tolerance is `1e-12`.
A scaled norm calculation avoids intermediate squared-norm overflow. Negative or
nonfinite tolerances are rejected. Tolerance is dimensionless and caller-selected;
a large tolerance weakens the unit-length check. These predicates return bool,
do not throw, mutate, normalize, clamp, or repair inputs. A zero quaternion is
invalid even with a large tolerance.

Every future numerical boundary must validate inputs and outputs appropriate to
its contract, including overflow after arithmetic. Predicates cannot make mutable
Eigen values permanently valid and must never read uninitialized values. Direction
unit-length checks, angle ranges, frame compatibility, and operation-specific
error reporting remain the consuming API's responsibility. Future operations must
report invalid input explicitly rather than silently replace it with identity or
zero. M1.3 uses `std::optional` for failures and explicitly documents correction of
accepted unit-norm drift; M1.4 preserves this optional failure interface. No
universal epsilon or assert-only enforcement is introduced. M1.2 predicates do
not normalize; M1.3 rotation operations follow their documented bounded correction
policy.

## Example (implemented API)

```cpp
#include <aerotwin/math/Types.hpp>

const aerotwin::Vector3 position_world_m(1.0, 2.0, 3.0);
const aerotwin::Vector3 displacement_world_m(0.0, 0.0, 1.0);
const aerotwin::Vector3 next_position_world_m = position_world_m + displacement_world_m;
const aerotwin::Quaternion q_world_from_body = aerotwin::Quaternion::Identity();
const bool valid = aerotwin::isFinite(next_position_world_m)
                && aerotwin::isUnitQuaternion(q_world_from_body);
```
