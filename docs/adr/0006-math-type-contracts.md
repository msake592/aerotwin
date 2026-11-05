# ADR 0006: Use Eigen aliases with explicit mathematical contracts

Status: Proposed (M1.2 implemented; pending review)

## Context

M1.1 supplies Eigen. Later quaternion, transform, and simulation work needs shared
precision, units, frame direction, point semantics, and invalid-input rules.

## Decision

Use `aerotwin::Vector3 = Eigen::Vector3d` and
`aerotwin::Quaternion = Eigen::Quaterniond` in one public math header, with finite
vector and unit-quaternion predicates. Adopt the
[mathematical contracts](../architecture/math-contracts.md), SI units, world ENU,
body FLU, and Hamilton WXYZ interchange. Keep transform signatures a documented
draft until M1.4; do not ship unimplemented declarations.

## Rationale and trade-offs

Aliases match the existing small public namespace and preserve Eigen expressions
without wrappers or additional dependencies. Strong unit/frame/point wrappers
could catch misuse at compile time, but impose a larger API before consumers
exist. Names and documented boundaries carry these semantics for now. Eigen raw
quaternion storage differs from interchange order, requiring named accessors.
Mutable aliases cannot enforce invariants; boundary validation remains explicit.
Predicates reject invalid data without silently normalizing it. Quaternion algebra
and transformation implementations remain in M1.3 and M1.4 respectively.

## Consequences

Consumers must initialize values and declare units, origin, frame, and provenance.
Tests cover type identity, XYZ access, WXYZ constructor versus XYZW storage,
finite values, norm tolerance, and invalid quaternion rejection. They do not claim
rotation correctness or frame conversion coverage. Revisit strong types if actual
consumers demonstrate recurring semantic mistakes.
