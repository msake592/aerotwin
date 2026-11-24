# M3.3-B Docker transport and clock experiment

## Scope and provenance

This opt-in ARM64 experiment reuses the M3.2.2 compiled runtime. It starts
three separate services on an internal Docker bridge network. PX4/Gazebo,
the XRCE Agent, and the Jazzy observer have separate process, IPC, and
network namespaces. Services publish no host ports. Each service is limited
to two CPUs and 2 GiB RAM.

The recovered runtime snapshot is
`sha256:7212db2bb758fb941de919bbc5077907de0f773c94bcd415d703383031bf93ef`.
It was committed from the preserved `aerotwin-m322-build` container, originally
based on
`px4io/px4-sitl-gazebo@sha256:29a5a1b5af77e5d39b67bc6664d36acead63f9dfb829475216522b1feec8036e`.
This local snapshot requires the retained artifacts. A complete source image
build recipe remains a separate reproducibility deliverable.

The inventory verifies PX4 v1.17.0, Agent v2.4.3, the pinned message checkout,
the retained binary hashes, required Gazebo plugins and assets, installed
message definitions, Jazzy, and Gazebo 8.15.0. It records resolved Agent
dependency SHAs and installed ROS/Gazebo package revisions. See the
[M3.2 guide](m3-sitl-runtime.md#reference-candidates) for the source pins.
The PX4 and Agent binary SHA-256 values are respectively
`babe228be6179e8f7abf8af0912ee6b6fae701e9e18cbacfb4d092670ac74402`
and `4cee94fb122336a32c9479e4fe2656bee80d7f445abd08a6534be725167ebc0b`.

## Run from the checkout

Activate the editable Python environment. Docker Desktop and its Compose
plugin must be running. Choose a new, absolute evidence directory outside the
checkout on persistent storage. The CLI rejects an existing destination.

```sh
python -m aerotwin_experiments.m3_docker \
  --image sha256:7212db2bb758fb941de919bbc5077907de0f773c94bcd415d703383031bf93ef \
  --evidence "$EVIDENCE_DIRECTORY"
```

If the original snapshot is unavailable but the preserved build container
exists, recover it and verify its artifacts before making a new snapshot.

```sh
docker inspect aerotwin-m322-build
docker start aerotwin-m322-build
docker commit aerotwin-m322-build aerotwin/m33b-runtime:recovered
docker image inspect aerotwin/m33b-runtime:recovered --format '{{.Id}}'
```

Use the returned immutable image ID with the CLI. Snapshotting preserves
installed artifacts and caches. The role entrypoints explicitly bypass the
image's shell and PX4 entrypoint.

A short-lived inventory job verifies artifacts before the three runtime services start.
The coordinator expands and validates
`simulation/docker/compose.yaml`, inventories the runtime, and performs two
independent sessions. Each has a UUID project, Gazebo partition and PX4
namespace. Gazebo discovery stays on loopback within the simulator container.
DDS uses domain 55 across the Docker subnet. The simulator resolves
`xrce-agent` through Docker DNS and restarts its client with an explicit
numeric address and UDP port 18888. The Agent and observer resolve each other
for unicast RTPS discovery, using UDP transport and domain-derived discovery
port 21160. Their generated XML profiles are included in the evidence.

The coordinator reuses the M3 preflight bounded subprocess handling,
owned-process cleanup, Gazebo statistics parsing and odometry validation.
Regular CI exercises mocked failure paths and requires no robotics packages.
Exit 0 accepts two transport sessions with clean cleanup and removal. Exit 1
reports an executed transport or cleanup failure. Exit 2 reports a Docker or
image prerequisite failure. Clock results and adapter readiness remain
separate report fields.

## Evidence and recovery

Each session saves image identity, deployment configuration, package/source
inventory, resolved network topology, firmware output, offered/requested QoS,
publisher GID, receipt and source timestamps, process exit states, bounded
child logs, ULogs, and a JSON report. Observer samples are flushed to JSONL.
Clock heartbeat records continue during pauses without new odometry.

Readiness and commands have explicit timeouts. SIGTERM requests scoped
cleanup. A failed acceptance condition remains in the report. Shutdown
stops the observer before the simulator and Agent, checks all exit states
and child process cleanup, then removes only that UUID project's containers
and network. The original build container and image remain available.

After interruption, inspect the report and deployment files and
`docker ps -a --filter label=com.docker.compose.project=<project>` before
launching another run. A coordinator still executing should finish its
owned session. If it has exited while services remain, use the recorded
Compose project and environment from `deployment.json` to stop it and save
container logs before running `down`. A fresh run requires a new evidence
directory. The retained source build has no dependency on the coordinator.

## Observed transport and reset behavior

Default multicast discovery and an initial UDP-only multicast attempt timed
out while PX4 was connected to the Agent. Explicit DNS-resolved unicast peers
established discovery and decoded telemetry. This evidence establishes the
unicast configuration used here. The underlying multicast failure remains
unresolved.

Two independent sessions demonstrated a connected PX4 client, typed
VehicleOdometry discovery, finite decoded position/velocity/quaternion values,
ordered timestamps, one stable publisher, and clean service shutdown. The
publisher offered best-effort/transient-local QoS, and the observer requested
best-effort/volatile/keep-last depth five. Compatibility reported a warning
about automatic versus system-default liveliness. Received traffic passed the
semantic checks. VehicleOdometry remains a PX4 internal estimate.

The final normal observation windows contained 442 and 444 valid ordered
samples. Resume windows contained 447 and 438. Both pause windows and both
post-reset windows contained zero new samples. The median difference between
reception wall time and wire timestamps was about 5 ms during normal operation,
and about 5.25 seconds after pause/resume. That difference includes clock
mapping effects and cannot be used as a transport latency measurement.

Normal progression and resume produced ordered traffic. Pausing held Gazebo
and PX4 sensor time while the observer's system-backed ROS clock continued.
Time-only reset moved Gazebo time backward and resumed the world. During the
bounded post-reset window, PX4 sensor values retained their pre-reset times
and new odometry stopped. XRCE client status remained connected. In-place
coupled reset is **BLOCKED**. The independent second session demonstrates
a controlled full restart with a new publisher and simulation origin.

The existing reference runner's preserve-pause reset gate remains
conservative. This experiment separately checks the observed Harmonic 8.15.0
contract of backward time movement followed by progression, and preserves
the stalled PX4 telemetry as a coupled-reset limitation.

PX4's default localhost client first logged a failed Agent ping. The role then
stopped that client and established the explicitly addressed Docker session.
Both sessions also logged the expected missing-GCS arming warning.

The server logs include an unavailable optional GstCameraSystem plugin and
SDFormat warnings for `gz_frame_id`. Camera functionality was outside the
x500 IMU/odometry experiment.

## Clock domains and proposed adapter contract

| Domain | Representation and origin | Evidence and boundary |
| --- | --- | --- |
| Gazebo world | Integer seconds/nanoseconds from world start | Advances during normal operation, holds on pause, rolls back on time reset |
| PX4 uORB | Integer microseconds on the SITL clock | Pinned GZBridge sets the PX4 clock from Gazebo clock callbacks; IMU publication uses `hrt_absolute_time()` |
| DDS publication/acquisition | Integer microseconds after XRCE time adjustment | Generated VehicleOdometry serialization adds `session.time_offset / 1000` to both fields |
| XRCE offset | Session nanoseconds and Timesync microseconds | The pinned callback negates the estimated offset and converts it to nanoseconds; convergence and changes require separate observation |
| ROS node | ROS clock with `use_sim_time=false`, backed by system time | No Gazebo-to-ROS `/clock` bridge participates in this experiment |
| Reception | Linux monotonic nanoseconds and system wall nanoseconds | Samples retain both; observation phases compare monotonic readings only after verifying the Linux boot identity and matching namespace time offsets |
| Host correlation | macOS monotonic and wall nanoseconds | Retained separately; host monotonic values are excluded from Linux phase comparisons |

The source mechanism is established by the pinned
[GZBridge clock callback](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/simulation/gz_bridge/GZBridge.cpp),
[DDS serialization dispatch](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em)
and [XRCE synchronization callback](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/uxrce_dds_client.cpp).
The generated serializer adds the offset to both `timestamp` and
`timestamp_sample`. Magnitude alone supplies no clock mapping.

**M3.4 timestamp adapter readiness is BLOCKED.** The proposed contract requires
the following evidence before estimator input is enabled.

- Carry run ID, simulation epoch, source clock identity, synchronization mode,
  origin and units with every sample. Preserve acquisition, publication and
  reception times separately.
- Convert verified microseconds by checked integer multiplication by 1,000.
  Check signed 64-bit range and valid offsets before creating core timestamps.
  Convert ROS seconds/nanoseconds using checked integer arithmetic.
- Open a new epoch for an explicitly orchestrated full restart. Clear queued
  samples and mapping state. Validate new source and publisher identity before
  accepting observations. A stale packet or estimator reset counter alone
  cannot establish a new simulation epoch.
- Quarantine backward, duplicate and reordered samples. Report missing samples
  with sequence/time gaps, and reject invalid or unknown timestamps. Best-effort
  delivery permits gaps. Ordering within each recorded window supplies no
  guarantee about later traffic.
- Agent reconnection and synchronization changes invalidate the offset mapping
  until revalidated. Detect publisher changes and convergence loss explicitly.
  This package validates new full sessions, while isolated Agent reconnection
  remains unverified.
- A Gazebo-only rollback cannot safely reset the coupled clock contract from the
  available evidence. Use controlled full restart for further development.
  Post-pause synchronization and per-message offset correlation require further
  evidence before conversion into the simulation epoch.
- Preserve ENU/FLU, Hamilton quaternion direction, covariance ordering and truth
  isolation at the adapter boundary. PX4 estimates remain separate from
  simulator truth, sensor observations and AeroTwin estimates.

Multi-container ARM64 transport can pass independently of clock mapping.
Ubuntu AMD64 reference validation remains pending.

## Regression verification

Local verification on the M3.3-B implementation passed 67 pytest cases,
including 29 portable Docker/clock cases, and 230 CTest cases in each
configuration. The live two-session evidence is separate from these tests.

```sh
cmake --preset dev-debug
cmake --build --preset dev-debug --parallel 2
ctest --preset dev-debug
cmake --preset dev-release
cmake --build --preset dev-release --parallel 2
ctest --preset dev-release
python -m pytest -p no:cacheprovider
ruff check .
ruff format --check .
python -m pip check
git diff --check
```

The CLI's inventory-only Compose job and expanded configuration were also
executed successfully after the live tests. Hosted CI results belong to the
PR checks and should be inspected independently.
