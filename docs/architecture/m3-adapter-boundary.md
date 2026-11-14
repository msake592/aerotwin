# M3 adapter boundary

Place ROS transport and conversions under `ros2_ws/src/` and simulator assets
under `simulation/`. Preserve the standalone core, Hamilton quaternions, ENU/FLU,
integer nanoseconds and epochs, and the validated 6D state ordering
`[px, py, pz, vx, vy, vz]`. See [ADR 0001](../adr/0001-core-independent-of-ros.md),
[ADR 0004](../adr/0004-simulation-time.md), and [measurement contracts](measurements.md).

| Stream | First integration use |
| --- | --- |
| Gazebo world pose and velocity | Truth for alignment and scoring. Verify model reference point, world axes, origin and clock. |
| PX4 SensorGps | Candidate measurement. Verify fix, origin, height datum, sample time and uncertainty semantics. Convert WGS84 geodetic via ECEF to a declared run-local ENU origin. Require a verified mapping to Gazebo's origin. An accuracy scalar alone does not establish a covariance model. |
| PX4 SensorCombined or simulator IMU | Diagnostic. Verify FRD, clipping, integration periods and accelerometer/gyroscope sample times. Integrated VehicleImu increments are a separate contract. |
| PX4 VehicleOdometry | PX4 internal estimate for comparison. Treat frame enums, reset counters, quality and reported variances as source metadata. |
| AeroTwin estimator | Uses only validated observations and its own state. Truth and PX4 internal estimates feed comparison paths. |

Match actual ROS topic names, including namespaces or version suffixes, to
the [tagged DDS registry](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/uxrce_dds_client/dds_topics.yaml).
Keep run ID, epoch, sensor identity, source clock, sample and receipt times,
origin, units and quality metadata. Unknown or invalid semantics block
conversion to the core typed measurement factories.

For aligned origins and headings use NED → ENU `A(x,y,z)=(y,x,-z)` and body
FRD → FLU `B(x,y,z)=(x,-y,-z)`. Translate positions for different origins.
Body-frame velocity requires a verified attitude. World-fixed FRD with an
arbitrary heading requires an explicit world alignment. The active rotation
is `R_enu_from_flu = A * R_ned_from_frd * Bᵀ`. Check external WXYZ ordering,
basis rotations, sign equivalence and float-to-double precision. Any
adapter normalization must be bounded and tested against the core quaternion
contract. See [coordinate frames](coordinate-frames.md#px4-boundary-nedfrd).

Use `C P Cᵀ` with `A` for world GPS, `B` for body sensor blocks, and
`diag(A,A)` for aligned position/velocity. A nonlinear geodetic conversion
needs a justified local Jacobian. Missing cross terms and unknown covariance
remain explicit; do not present inferred covariance as measured.

## Clock and transport gates

- Candidate policy uses Gazebo as the clock authority, a unidirectional
  ROS bridge for `/clock`, and explicit `use_sim_time` on participating nodes.
  Verify PX4 stepping and `UXRCE_DDS_SYNCT=false` for this clock mode; the
  OS-clock mode has different assumptions. Document the capture clock and
  epoch offset of every selected field.
  [Tagged PX4 ROS guide](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/docs/en/ros2/user_guide.md#ros-gazebo-and-px4-time-synchronization)
- Convert microseconds with checked integer multiplication by 1,000; convert
  ROS seconds/nanoseconds with checked integer arithmetic. Preserve source
  precision, separate publication and arrival from acquisition, and reject
  unknown offsets, negative elapsed values and overflow. Confirm pause,
  non-real-time stepping and rollback on a real session.
- A confirmed clock reset opens a new epoch, drops old queues, and resets
  mapping and estimator initialization. A stale packet or PX4 estimator
  reset counter alone does not reset the simulation clock. Diagnose missing,
  stale, duplicate and reordered samples and preserve capture time.
- Start single-host with explicit domain, RMW, and compatible offered and
  requested QoS. Measure discovery, rate, reliability, durability and depth
  separately from clock alignment. Keep updates and comparisons disabled
  until clock mapping and measurement semantics are demonstrated.

## M3.2 acceptance

Pass the asset-free Gazebo probe on a suitable Ubuntu host, then validate
isolated tagged PX4 x500 SITL, a matching XRCE Agent and real ROS data.
Capture package versions, source/submodule SHAs and image digest or host
manifest. Check repeatable startup and bounded shutdown with no orphan
processes. Record exported topics, clock behavior across pause and reset,
message definitions, QoS, and unresolved origin/covariance questions.
This evidence gates production adapters. Armed flight and estimator changes
are outside M3.2 host validation.
