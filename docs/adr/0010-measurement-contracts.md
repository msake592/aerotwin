# ADR 0010 — Separate sensor observations from simulator truth

Status is pending technical review.

## Decision

Define owned GPS and IMU observations with validated acquisition metadata and
covariance. Keep the truth-facing sensor interface in a separate header so
measurement consumers can include their contracts independently of
`VehicleState`. Reuse the simulation timestamp type for acquisition time.

## Rationale and consequences

The separate boundary lets future sensor models consume simulator truth while
estimators operate on observations. GPS uses one homogeneous position
covariance. IMU uses separate homogeneous specific-force and angular-rate
covariances, which keeps incompatible units in separate blocks. Pair-local symmetry and
variance-normalized PSD checks use a dimensionless `1e-12` tolerance and keep
large variances from masking defects in smaller directions. Zero-variance axes
require exactly zero cross-covariances, and singular PSD blocks are valid. A future consumer that needs cross-covariance will
require an explicit contract extension. The
[measurement contract](../architecture/measurements.md) defines frames, units,
numerical validation, and reset behavior.
