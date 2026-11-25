# M3.3-C clock correlation and epoch enforcement

M3.4 remains **BLOCKED**. The validation infrastructure rejects unproven
mappings and preserves temporal boundaries. Exact source-to-wire message
identity remains unavailable in the exercised interfaces.

## Source semantics

The runtime remains PX4 v1.17.0 at
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, Gazebo 8.15.0,
Agent v2.4.3 and firmware-compatible Jazzy messages.

| Field/domain | Source-confirmed semantics |
| --- | --- |
| VehicleOdometry.timestamp | Microseconds, PX4 publication timestamp |
| VehicleOdometry.timestamp_sample | Microseconds, underlying sample timestamp; repeated values cannot supply unique message identity |
| PX4 IMU/uORB time | Simulation clock through `hrt_absolute_time()` |
| Gazebo clock | Seconds/nanoseconds since world start |
| XRCE wire timestamps | Publication and acquisition fields adjusted by the session offset |
| ROS reception | System-backed ROS clock plus independent Linux monotonic reception capture |

The pinned `src/modules/simulation/gz_bridge/GZBridge.cpp::clockCallback`
sets PX4 time from Gazebo. `imuCallback` assigns internal publication/sample
times from `hrt_absolute_time()`. The generated
`build/px4_sitl_default/uORB/ucdr/vehicle_odometry.h` adds `time_offset`
to both odometry timestamps. The dispatch in
`src/modules/uxrce_dds_client/dds_topics.h.em::SendTopicsSubs::update`
passes `session.time_offset / 1000`. The client's `on_time` callback
negates the Timesync estimated offset and scales it to nanoseconds.
`msg/versioned/VehicleOdometry.msg` defines the field units.

These mechanisms establish a transformation in source code. They do not
establish a measured per-message error bound after pause or reconnection.

## Validation contract

`m3_clock.py` provides validation-layer observations, candidate correlation
statistics and a mapping state machine. Every observation carries epoch,
mapping generation, publisher, capture identity, raw timestamp, domain,
units, semantics and reception monotonic time.

Correlation accepts only independently verified, unique identities. It
rejects ambiguous identities, mismatched epochs/generations and unverified
pairing. Receipt-row IDs identify capture records only. The odometry CLI
prints rounded vectors and lacks a shared wire message ID. Nearest timestamps
and `timestamp_sample` alone remain insufficient evidence. Candidate offset
residuals measure observed variation, and the report leaves uncertainty
unknown. Live capture therefore reports unmatched rows and a blocked mapping.

A mapping progresses from UNINITIALIZED through VALIDATING to VALID only
after explicit validation for its current epoch and generation. Synthetic
unit fixtures exercise that path. The live tool leaves mappings invalid.
Microseconds convert using checked integer multiplication by 1,000. Negative,
zero, noninteger, overflowing and out-of-range mapped timestamps are rejected.

Publisher/domain changes, duplicate or backward observations, synchronization
changes and Agent reconnection invalidate the mapping. A fresh generation
requires publisher and mapping revalidation. Reconnection preserves the
simulation epoch until a separate full runtime restart is explicitly
orchestrated. Old epoch/generation observations cannot be reused.

A full restart creates a distinct simulation epoch and invalidates the old
mapping. The validator exposes no estimator time-delta calculation. In-place
reset invalidates the mapping and is rejected. ENU/FLU, Hamilton quaternions,
covariance ordering and truth isolation remain the adapter boundary contracts.
VehicleOdometry represents a PX4 estimate.

## Retained evidence normalization and live scope

The earlier two reconnection sessions recorded their post-recovery phase under
`time_only_reset`. The explicit experiment provenance states that no Gazebo
reset was requested. `normalize_reconnection` returns a derived
`agent_reconnection` record while preserving the raw schema and observations.
Publisher changes correctly retain the original continuity failure. Normal,
pause, resume, Agent interruption/recovery and controlled full restart remain
distinct operations.

The focused CLI uses the existing Compose coordinator and immutable snapshot.
It normalizes retained reconnection reports, collects one normal clock window
per epoch across two controlled full sessions, and emits correlation reports.
It omits the previous pause/reset campaign. It never initializes an unproven
live mapping.

```sh
python -m aerotwin_experiments.m3_clock_validation \
  --image sha256:7212db2bb758fb941de919bbc5077907de0f773c94bcd415d703383031bf93ef \
  --retained /private/tmp/aerotwin-m33c-evidence \
  --evidence "$NEW_EXTERNAL_EVIDENCE_DIRECTORY"
```

A completed capture writes `complete`. Inspect the execution session and log
before retrying. Use a new directory for any additional attempt. A successful
capture exit confirms cleanup, while mapping readiness is reported separately.
The original build container and snapshot remain preserved.

## Acceptance and next requirement

| Gate | Decision |
| --- | --- |
| Timestamp source semantics | PASS for the scoped fields |
| Exact per-message correlation | BLOCKED |
| Live mapping validity and pause/resume accuracy | BLOCKED |
| Mapping invalidation | PASS in deterministic tests |
| Epoch/generation isolation | PASS in deterministic tests; live epoch identities captured separately |
| Agent reconnection | Recovery observed; continuity requires revalidation |
| In-place coupled reset | BLOCKED |
| Ubuntu AMD64 reference | NOT_RUN |

The next prerequisite is a source-side capture with a defensible shared
identity, or a bounded correlation mechanism with quantified uncertainty.
That evidence must cover pause/resume and synchronization recovery before
production M3.4 timestamp conversion is enabled.

## Verification results

The focused live capture collected 572 and 574 wire observations across two
distinct runtime identities. Both sessions passed transport and cleanup.
Zero observations qualified for exact source-to-wire pairing. Offset,
residual error bounds and synchronization precision remain unmeasured.

The prior Agent experiment remains sufficient for the invalidation policy.
Its publisher change is evidence for mandatory revalidation, while telemetry
recovery alone supplies no continuity proof.

Local verification used the following commands with an editable development
installation. The full suite passed 89 pytest cases, including 22 focused
clock cases. Debug and Release each passed 230 CTest cases. Docker Compose
configuration validation occurred through the two short session deployments.

```sh
python -m pytest -p no:cacheprovider
ruff check .
ruff format --check .
python -m pip check
ctest --preset dev-debug
ctest --preset dev-release
git diff --check
```

## M3.3-D follow-up

The [source-to-wire verification](m3-source-wire.md) subsequently configured
runtime ULog capture and matched unique invariant payloads. Two independent
sessions measured the serializer offsets. Absolute synchronization uncertainty
and estimator timing tolerances remain unresolved, so production mapping
initialization remains blocked.
