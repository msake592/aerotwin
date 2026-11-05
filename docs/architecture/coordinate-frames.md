# Coordinate frame conventions

Status: M1.2 type and unit contracts implemented; transformations remain planned. See
[mathematical contracts](math-contracts.md) for public types, validation, and API draft.

| Frame | Axes | Intended use |
| --- | --- | --- |
| World ENU | X east, Y north, Z up | Canonical world frame for core-facing simulation contracts |
| World NED | X north, Y east, Z down | PX4-facing aerospace integration |
| Body FLU | X forward, Y left, Z up | Core-facing body convention |
| Body FRD | X forward, Y right, Z down | PX4-facing body convention |
| Sensor-local | Declared per sensor | Measurement generation and interpretation |

All listed conventions are right-handed. ENU/NED world frames and FLU/FRD body frames are different concepts and must not be used interchangeably. Gazebo world alignment and geodetic origin must be configured explicitly; a simulator's world axes must not be assumed to mean east and north automatically.

Future adapters must explicitly convert between declared source and destination frames. Interfaces must document vector expression frame, pose reference frame, transform direction, sensor extrinsics, and covariance ordering. For interchange, serialize unit quaternions as `[w, x, y, z]` using the Hamilton convention. A body attitude actively maps body-frame vectors into world coordinates using `v_world = q * v_body * conjugate(q)`, with vectors embedded as pure quaternions. The inverse maps world vectors into body coordinates. Serialize components explicitly at library and adapter boundaries.

Use metres for position, metres per second for velocity, radians for angles, and radians per second for angular rates. Declare the expression frame and units for noise and covariance as well. Use [run-relative simulation time](../adr/0004-simulation-time.md) for observations and states. Rotation and transformation algorithms and their tests remain deferred to M1.3/M1.4.
