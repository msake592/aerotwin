# Coordinate frame conventions

Status: proposed conventions and interface requirements. No transformations are implemented.

| Frame | Axes | Intended use |
| --- | --- | --- |
| World ENU | X east, Y north, Z up | Proposed canonical world frame for core-facing simulation contracts |
| World NED | X north, Y east, Z down | PX4-facing aerospace integration |
| Body FLU | X forward, Y left, Z up | Proposed core-facing body convention |
| Body FRD | X forward, Y right, Z down | PX4-facing body convention |
| Sensor-local | Declared per sensor | Measurement generation and interpretation |

All listed conventions are right-handed. ENU/NED world frames and FLU/FRD body frames are different concepts and must not be used interchangeably. Gazebo world alignment and geodetic origin must be configured explicitly; a simulator's world axes must not be assumed to mean east and north automatically.

Future adapters must explicitly convert between declared source and destination frames. Interfaces must document vector expression frame, pose reference frame, transform direction, sensor extrinsics, and covariance ordering. Attitude interfaces must state quaternion component order, rotation direction, and active/passive interpretation before implementation; those representation choices are not finalized here.

Use SI units by default: metres, seconds, radians, and their derived units. Every observation and state must carry simulation time with a declared epoch/clock, and integration must handle resets explicitly. Noise and covariance assumptions must identify units and expression frames.

Frame and timestamp contracts must be reviewed and tested before any transformation or sensor fusion is introduced. Do not mix conventions implicitly or use wall-clock timestamps as simulation time.
