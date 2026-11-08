# Deterministic GPS simulation

M2.3 implements the GPS generation side of the
[measurement contract](measurements.md). Include `<aerotwin/sensors/GpsSensor.hpp>`
and link `aerotwin_core`. A `GpsSensor` implements
`SensorModel<GpsMeasurement>`. It borrows a validated `VehicleState` snapshot
for each call and returns an owned measurement. Estimators consume the
measurement contract independently of the truth-facing generator.

## Configuration and measurement

`GpsSensorConfig::create(id, covariance, seed)` validates the identifier and
3×3 position covariance through the M2.1 factories. It stores the accepted
symmetric covariance in world ENU m² and caches a
[Gaussian sampling factor](gaussian-noise.md) for that covariance. It owns the
identifier and the caller-supplied 64-bit seed. Invalid configurations return
an empty optional.

The measured position is the truth body origin in local world ENU metres plus
one world ENU Gaussian noise vector. Zero covariance preserves the exact
position. Each measurement carries the configured canonical covariance, with
no per-sample estimation or adjustment. As in M2.2, the sampling factor projects
tolerated negative correlation eigenvalues to zero. The covariance metadata
continues to describe the validated configuration.

## Time and replay

The caller supplies acquisition time and a snapshot with exactly the same epoch
and elapsed integer nanoseconds. A mismatch, including negative elapsed time,
is rejected before drawing. The generator preserves the supplied timestamp
and performs no interpolation, extrapolation, or clock queries. Matching
timestamps do not establish run provenance. The caller supplies truth from
the intended run.

Each sensor owns its stream. Instances constructed from the same configuration
start independently from the same seed. Copying a sensor snapshots its stream.
`reset()` explicitly restarts from the configured seed. Changing clock epochs
does not reset the stream. Replay requires the same configuration, input order,
executable, and floating-point environment.

## Failure and scope

A valid sampling attempt consumes three normal draws, including at zero
covariance. Noise sampling failure or measurement factory rejection returns an
empty optional after those draws. The measurement factory rejects a non-finite
sum without clamping. The truth factory already prevents non-finite input
positions, and the generator checks again before drawing. Under default
round-to-nearest binary64 arithmetic, the finite M2.2 draws and covariance scales
are too small to overflow even when added to the maximum finite position.
Small offsets at that scale can round away.

This is local Cartesian position simulation. Geodetic conversion, satellite
and propagation effects, bias, drift, scheduling, latency, and delivery
remain future work.
