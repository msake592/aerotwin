# Deterministic IMU simulation

M2.4 provides `ImuSensor`, `ImuSensorConfig`, and `ImuKinematics` in
`<aerotwin/sensors/ImuSensor.hpp>`. The IMU is colocated with the vehicle
reference point. The caller supplies a validated `VehicleState` for orientation and
body angular velocity, and an owned `ImuKinematics` value for finite linear
acceleration of the reference point in world ENU m/s². A future dynamics or
motion provider supplies that acceleration. The model performs no finite
differencing or kinematic estimation.

The IMU-specific `measure(truth, kinematics, acquisition_timestamp)` API adds
one required physical input beyond the existing two-argument `SensorModel`
interface. It leaves the GPS interface and the estimator-facing
[measurement contracts](measurements.md) intact.

## Physics and configuration

The accelerometer measures specific force, which is world linear acceleration
minus gravity, expressed in body FLU. The sensor computes
`f_body = R_world_from_body^T (a_world - g_world)` using the existing checked
world-to-body vector transformation and the active Hamilton quaternion.
There is no translation in the transformation. The default gravity is
`(0, 0, -9.80665)` m/s² in world ENU. Callers can supply a different finite
gravity vector. A level stationary snapshot gives upward specific force,
and acceleration equal to gravity gives zero specific force before noise.

The gyroscope measures the snapshot's body FLU angular velocity in rad/s.
The sensor adds independent-stream Gaussian noise to the two ideal FLU
vectors. Configuration validates and retains the canonical M2.1 covariance
blocks in m²/s⁴ and rad²/s² and caches the corresponding M2.2 factors. Zero,
singular, diagonal, and correlated within-channel covariances are supported.
The two blocks carry no cross-covariance. Tolerated negative correlation
eigenvalues are projected for sampling under the
[M2.2 policy](gaussian-noise.md). The measurement retains the configured
canonical covariance metadata.

## Time and replay

Truth, acceleration, and acquisition timestamps must match in both epoch
and integer elapsed nanoseconds. Matching timestamps do not establish run
provenance. The caller supplies both inputs from the same intended run.
The output preserves the acquisition timestamp exactly.

Configuration requires two distinct caller-supplied 64-bit seeds, named for
their channels. Each sensor owns both streams. Copying snapshots both streams
at their current positions. `reset()` restarts both from their configured seeds.
Epoch changes never implicitly reseed. Replay requires the same configuration,
input sequence, executable, and floating-point environment.

## Failures and numerical limits

Invalid input construction, timestamp mismatches, and an unrepresentable
specific-force subtraction or rotation fail before sampling. The truth
factory and checked rotation enforce the orientation contract, and angular
velocity must be finite. Once sampling starts, each channel consumes three
normal draws, including at zero covariance and on any subsequent failure.
Non-finite noise or measurement results return an empty optional. The model
does not clamp outputs.

Subtraction and rotation can overflow with finite extreme inputs. The checked
transform rejects non-finite intermediates or output according to the existing
rotation policy. In default round-to-nearest binary64 arithmetic, M2.2 bounds
each noise component below `7.3e155` in the channel's units even with maximum
finite variances. This is well below the approximately `9.98e291` rounding
margin above the largest finite component. Small offsets can round away.
Bias, saturation, quantization, lever arms, scheduling, and estimation are
outside M2.4.
