# ADR 0007: Checked quaternion rotation operations

Status: Proposed (M1.3 implemented; pending review)

## Context

M1.2 supplies mutable Eigen aliases and validation predicates, but no checked
quaternion operations. Later frame transformations need explicit order, active
rotation semantics, normalization policy, and failures.

## Decision

Expose four compiled functions in `<aerotwin/math/Rotations.hpp>`:
`normalizeQuaternion`, `composeRotations`, `inverseRotation`, and `rotateVector`.
Use `std::optional` for explicit failure without adding dependencies. Follow the
[rotation contract](../architecture/quaternion-rotations.md): Hamilton WXYZ,
active rotations, and `after * before` composition.

Normalize arbitrary finite nonzero quaternions only through the explicit function.
Other operations require the existing absolute norm tolerance `1e-12`, remove
accepted roundoff drift, and return normalized rotations. Scale quaternion
normalization to bound its norm calculation. For vector rotation, retain the
unscaled matrix-vector result in every finite row; retry only nonfinite rows with
scaled intermediates to recover from partial-sum overflow and cancellation. This
avoids unconditional scaling that can erase representable small components in
unrelated rows. Reject outputs still nonfinite after retry. Double rounding,
underflow, and cancellation remain subject to the documented accuracy limits.

## Rationale and trade-offs

The four functions preserve Eigen interoperability and keep error handling small.
Optional results force callers to acknowledge failure, but do not identify its
cause. Exceptions and a custom error hierarchy are unnecessary for these basic
operations. A validated wrapper could carry invariants across calls, but is
premature before concrete consumers exist. Revalidation costs are accepted.
Correcting accepted norm drift maintains rotation semantics and limits accumulated
composition error; scaled nonunit inputs are never silently accepted.

## Consequences

Tests must establish multiplication order using noncommuting rotations, WXYZ
component handling, inverse round trips, norm preservation, extreme normalization,
and invalid-value/tolerance rejection. Default IEEE double behavior is assumed;
fast-math or flush-to-zero modes are not supported or enabled. Full transforms,
frame conversions, clocks, and state models remain outside M1.3.
