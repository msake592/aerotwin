# Sensor measurements

M2.1 adds validated C++ measurement values in
`<aerotwin/sensors/Measurements.hpp>`. Both factories return an empty optional
on invalid input. Measurements own their data and expose no field setters.

| Measurement | Observation | Covariance |
| --- | --- | --- |
| GPS | Local world ENU position in metres, with axes east, north, up | ENU position error in m² |
| IMU accelerometer | Specific force in body FLU in m/s², with axes forward, left, up | FLU specific-force error in m²/s⁴ |
| IMU gyroscope | Angular velocity in body FLU in rad/s | FLU angular-velocity error in rad²/s² |

GPS position uses the scenario's local ENU origin. Specific force is the
accelerometer quantity, conventionally inertial acceleration minus gravity
expressed in the body axes. It differs from world-frame linear acceleration.
All covariance matrices use the observation's axis order. The IMU stores two
marginal 3×3 blocks. The contract carries no cross-covariance and makes no
claim that accelerometer and gyroscope errors are independent. Neither
measurement carries an orientation. The existing active Hamilton WXYZ
convention still applies to any future explicit body/world transformation.

## Validation

`MeasurementAcquisition::create(id, timestamp)` accepts 1–64 ASCII letters,
digits, periods, underscores, or hyphens and requires nonnegative elapsed
nanoseconds. It owns the identifier. The accessor returns a borrowed view.
The GPS and IMU factories take a validated acquisition value and reject
non-finite observation components.

Each covariance must be finite and have nonnegative diagonal entries. A
zero-variance axis requires exactly zero off-diagonal entries in both its row
and column. Zero and singular PSD covariances are valid.

For positive variances, each off-diagonal pair uses its own scale: the maximum
of the two absolute covariance entries and the geometric mean of the two
variances. The pair must agree within `1e-12` relative to that scale. Accepted
pairs are averaged with an overflow-safe midpoint, and the result is stored
as an exactly symmetric matrix in its original units.

PSD is checked on the correlation matrix of the stored covariance. Each
off-diagonal entry is divided by the geometric mean of its two variances.
Mantissa/exponent arithmetic avoids forming an overflowing variance product
or rounding a subnormal geometric mean before division. Isolated zero-variance
axes use unit placeholders for this check. Non-finite normalized values and
correlation magnitudes above `1 + 1e-12` are rejected. The self-adjoint eigensolve
must succeed with finite eigenvalues and a minimum of at least `-1e-12`.
An unrelated large variance cannot relax either check. Very small normalized
entries may round to zero, and decisions near the tolerance remain subject to
floating-point rounding. Small negative eigenvalues within the correlation
tolerance are accepted without projection. Consumers requiring strict positive
definiteness must apply their own factorization policy.

## Time, ownership, and boundaries

- The truth timestamp identifies the time of a simulated state snapshot.
- The acquisition timestamp identifies the time at which a measurement was
  taken. It is stored as the existing `SimulationTimestamp`, with an explicit
  epoch and run-relative signed integer nanoseconds.
- A future delivery timestamp identifies when a consumer receives the
  measurement. Delivery does not change the recorded acquisition timestamp.

A reset advances the clock's epoch. Pipeline owners must keep epochs separate
and discard or explicitly route stale measurements. Epochs are local to one
clock history. Independent clocks or runs require separate routing or an
external run identifier. Factories validate timestamp values without consulting
a clock or verifying provenance.

`SensorModel<Measurement>` in `<aerotwin/sensors/SensorModel.hpp>` is the
generation-side interface for future GPS and IMU implementations. It exposes
sensor identity and borrows truth for a call at an explicit acquisition
timestamp. A successful call returns an owned measurement tagged with that
identity and timestamp. Implementations must reject mismatched truth times
or epochs. Callers own scheduling and clock advancement. Any future
stochastic inputs must be explicit, with no implicit clock, random state,
or telemetry export dependency in the interface.

Estimators include the measurement contracts directly and receive owned
measurements through a separate pipeline. Ground truth remains a generation
and validation input. Sensor physics, noise, asynchronous delivery, and
estimation are future steps.
