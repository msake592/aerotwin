# Quaternion and rotation mathematics

Status: M1.3 implemented; pending review, merge, and hosted CI. Full frame
transformations remain M1.4 work. Include `<aerotwin/math/Rotations.hpp>` and link
`aerotwin_core`. No dependencies beyond C++20 and the existing Eigen are added.

## API and failure policy

All functions are in `aerotwin`, take const references, return values, and do not
mutate inputs. Each returns `std::optional<T>`; `std::nullopt` means invalid input
or a nonfinite/unrepresentable output. Callers must check before dereferencing.
No fallback identity/zero, exceptions for input validation, or assert-only checks
are used. Invalid-input failures are not distinguished by an error enum in this
small API. Uninitialized Eigen objects remain the caller's responsibility.

| Function | Return value on success | Input contract |
| --- | --- | --- |
| `normalizeQuaternion(q)` | `Quaternion` | Finite coefficients, at least one nonzero |
| `composeRotations(after, before)` | `Quaternion` | Both inputs pass default `isUnitQuaternion` |
| `inverseRotation(q)` | `Quaternion` | Input passes default `isUnitQuaternion` |
| `rotateVector(q, v)` | `Vector3` | Default unit-quaternion check and finite vector |

### Normalization

Normalization is explicit for arbitrary scaled inputs. Divide coefficients by
their largest absolute component before computing the norm. This handles even
finite coefficients whose original norm would overflow, tiny normal values, and
subnormal nonzero values under the default floating-point environment. Zero and
nonfinite inputs are rejected. Quaternion sign is preserved; there is no W-positive
canonicalization. Components too small relative to others may round to zero, as
usual for double arithmetic.

Rotation operations first require `abs(norm - 1) <= 1e-12` using the existing
`kQuaternionNormTolerance`. This is an absolute, dimensionless **norm** tolerance,
not a squared-norm or angular tolerance. The API does not expose a way to loosen
it. Inputs outside it must be explicitly normalized by the caller. Accepted tiny
norm drift is removed before applying the operation; composition output is also
normalized to prevent accumulated drift. This bounded correction is part of the
operation contract, not acceptance of arbitrary scaled rotations. M1.2 predicates
still perform no normalization or mutation.

Double rounding means `1.0 + 1e-12` may lie slightly above the mathematical upper
boundary. Tests cover both sides of the tolerance, adjacent representable values,
and M1.2's exactly representable inclusive boundary. Numerical comparisons of
moderate-magnitude vector results use absolute `1e-12`; this is test-scale specific,
not a universal tolerance for all physical quantities or vector magnitudes.

### Active rotation, composition, and inverse

Hamilton quaternions use WXYZ construction/interchange; Eigen `coeffs()` remains
XYZW. Operations do not serialize raw coefficients. Positive rotations obey the
right-hand rule: positive 90 degrees around Z maps X to Y.

`rotateVector(q, v)` implements the active rotation
`v_out = q * (0, v_in) * conjugate(q)` with a normalized quaternion. Vector units
are unchanged and zero is valid. Implementation uses the equivalent rotation
matrix. It first computes the matrix-vector product **without vector scaling**.
Each finite output component is retained. This avoids losing a representable
small component merely because an unrelated input component is huge: identity
rotation preserves `(1e300, 1e-100, 0)`, including its Y component. Exact signed
permutation matrices also preserve representable small/subnormal components.
There is no identity-quaternion special case.

If a row produces a nonfinite value, only that row is retried after dividing the
vector by its largest absolute component, then rescaling the row result. Bounded
intermediates can recover a representable final component when unscaled partial
sums overflow before cancellation. The other finite rows are not recomputed or
scaled. Nonfinite input or an output still nonfinite after this retry is rejected.
A genuinely overflowed final component therefore returns `std::nullopt`; near the
double range boundary, coefficient and arithmetic rounding also affect this test.
This is a finite-result check, not an exact real-arithmetic representability proof.

Componentwise accuracy depends on the calculation, not just the final magnitude:

- Products and sums use double precision. Subnormal products can underflow, and
  adding a tiny term to a much larger term can round it away. Cancellation can
  expose those earlier errors; a mathematically representable result is not a
  guarantee of recoverable componentwise relative accuracy.
- A row using the overflow retry can lose terms during scaling and cancellation.
  The retry avoids unnecessary loss in other rows, but is not a compensated or
  arbitrary-precision dot product. A finite but cancellation-damaged row is not
  detected or recomputed.
- Quaternion/matrix coefficient rounding can mix a small fraction of a large
  component into another axis. Exact preservation in tests for identity and exact
  signed permutations does not imply exact preservation under arbitrary rotations.
- Gradual underflow and the default floating-point environment are assumed;
  flush-to-zero and fast-math modes are unsupported. Exact norm preservation at
  extreme magnitudes, full relative accuracy for subnormal results, and exact
  recovery after severe cancellation are not promised.

`composeRotations(after, before)` returns `after * before`: apply `before` first,
then `after`. Rotations generally do not commute. For compatible coordinate maps,
`q_target_from_source = composeRotations(q_target_from_mid, q_mid_from_source)`.
The caller owns frame compatibility; aliases carry no frame tags.

`inverseRotation(q)` returns the conjugate of its normalized input. It undoes the
active rotation and maps the output components back to the input frame. It is a
rotation inverse, not an inverse for arbitrary nonunit quaternions. Both `q` and
`-q` give the same vector rotation.

## Example

```cpp
#include <aerotwin/math/Rotations.hpp>
#include <cmath>

const double s = std::sqrt(0.5);
const aerotwin::Quaternion q_z90(s, 0.0, 0.0, s); // WXYZ
const auto composed = aerotwin::composeRotations(q_z90, q_z90);
if (composed) {
    const auto rotated = aerotwin::rotateVector(*composed, aerotwin::Vector3::UnitX());
    if (rotated) {
        // Approximately (-1, 0, 0): two positive quarter turns.
    }
}
```

## Relationship to M1.4

These are rotation primitives for the existing world ENU/body FLU contracts.
`rotateVector(q_world_from_body, v_body)` expresses body components in world axes
when the supplied attitude has that meaning. There is no hardcoded ENU/FLU basis
conversion. Rigid transforms, point translation, ENU/NED and FLU/FRD conversion,
origins, timestamps, and kinematics are not implemented. M1.4 can build on these
checked primitives and must choose its own concrete transform storage and failure
contract. See [mathematical contracts](math-contracts.md).
