# M3.3-F.1 source-time GPS qualification

**Overall GPS qualification: BLOCKED. M3.4 remains BLOCKED.** Source inspection,
a frozen protocol and deterministic offline checks are complete. No independent
GPS acquisition-event association was established at the offline gate, so no
live session was launched. **Zero of two permitted attempts were used.** No
production GPS adapter or acquisition accuracy claim is authorized.

## Baseline and preserved runtime

PR #29 was verified MERGED at
`7d58b86030a532f9680ac46707a82580c1d0bb76`, with all four hosted CI checks
successful. Origin/main was fetched, local main fast-forwarded and the clean
baseline used for `feature/m3-source-time-gps-qualification`.
[ADR 0011](../adr/0011-source-time-qualification-and-release-gate.md) is the
acceptance authority. No conflict was found between its stop conditions and the
F.1 request. The request approves the prototype envelopes and two 60-simulation-
second sessions, but does not waive independent acquisition-event evidence.

The preserved immutable local image is
`sha256:7212db2bb758fb941de919bbc5077907de0f773c94bcd415d703383031bf93ef`,
Linux ARM64. Read-only Docker inspection confirmed that it exists and
`aerotwin-m322-build` is stopped. No containers were running. The image/container
were not modified, started or removed. Existing pinned source copies and prior
artifact evidence were reused, without dependency installation or rebuilding.

| Component | Pinned evidence used, not a new live version check |
| --- | --- |
| PX4 v1.17.0 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| XRCE Client | `711aef423edd1820347b866d1e4164832df35d04`, project 2.4.0 |
| Agent v2.4.3 | `73622810d984349b80bbac0ef55fc0b694d62222` |
| px4_msgs / ROS | `86d8239e962f6939e05c3737784f60c02fa884db`, Jazzy, SensorGps definition |
| Simulator/model | gz-sim 8.15.0, model repository `b6127f4ec20de867e215fb5f78ae88b80f371909` |

This preserves earlier ARM64 runtime context. It supplies no new Ubuntu AMD64
robotics validation or current runtime parameter readback.

## Task A: source-level result

### Exact no-adjustment configuration

The proposed fresh-session settings are:

| Parameter | Required value | Source-level meaning |
| --- | ---: | --- |
| UXRCE_DDS_SYNCT | 0 | Client uses no-sync callback, skips synchronization loop and periodic timestamp resynchronization |
| UXRCE_DDS_SYNCC | 0 | Do not request Agent-based system-clock updates |
| SENS_GPS_MASK | 0 | Disable accuracy-based receiver blending |
| SENS_GPS_PRIME | 0 | Prefer raw receiver instance 0; this alone does not prohibit fallback |

Both XRCE parameters are declared reboot-required. A fresh full PX4 session,
recorded parameter readback and verified publisher inventory are required before
acceptance. These settings were **not applied** to the preserved container.

The XRCE library initializes `session.time_offset=0`. PX4 installs
`on_time_no_sync`, which sets that value to zero, when SYNCT is disabled.
Outgoing dispatch still reads the offset and generated SensorGps serialization
still adds it to both `timestamp` and `timestamp_sample`. The expected outgoing
operation is therefore exact integer addition of zero, rather than removal of
the serializer code. `time_utc_usec` is copied separately and is zero in this
simulation producer. It is not the estimator clock.

This conclusion is for outgoing GPS serialization. Incoming deserialization has
its own zero-time substitution/current-HRT clamping and is not qualified here.
Disabling SYNCT does not undo producer timestamp assignment, GPS blending,
quantization or other message transformations.

### Selected topic and receiver

ADR 0011 fixes the initial candidate to `/fmu/out/vehicle_gps_position`, with
`px4_msgs/msg/SensorGps`. Outgoing dispatch subscribes to uORB
`vehicle_gps_position` using `orb_subscribe`, instance 0. The vehicle GPS module
publishes a selected SensorGps observation, forwarding the selected raw
`sensor_gps` receiver. With blending enabled it can instead create weighted
sample/publication times, which are outside this contract.

The intended raw receiver is `sensor_gps` instance 0, with no other receiver
advertised and the same nonzero device_id locked throughout a capture. GZBridge
uses `PublicationMulti`, so the raw instance is the first available instance,
not an unconditional hardcoded zero. The protocol requires verifying inventory,
selected receiver and device identity before acceptance. A second receiver,
fallback, device change or ambiguous selection stops acceptance. No alternative
receiver path was silently selected. The actual running instance/device ID was
not measured and its qualification gate remains BLOCKED.

SensorGps contains float64 geodetic latitude/longitude and altitude fields,
float32 NED velocities, quality fields, device identity and integer microsecond
timestamps. Raw GPS is noisy: GZBridge adds position/velocity noise to the
Gazebo NavSat values. Source records for comparison must be from the **selected
vehicle_gps_position instance**, with raw sensor_gps instance 0 retained only
as selection/provenance evidence. Comparing raw instance 0 to a blended output
would test a different path.

### Acquisition path and independent-reference gap

The configured x500 NavSat sensor generates observations at **30 Hz**.
GZBridge receives the NavSat message, reads `hrt_absolute_time()` once at callback
entry, publishes a pre-noise global-position ground-truth diagnostic, adds GPS
noise and assigns the earlier HRT reading to both raw GPS timestamp fields.
The selected-receiver path forwards those fields. Publication time is therefore
a producer-assigned source-clock label, not an independently measured receipt
or sensor-generation time. Both fields being equal does not prove acquisition
registration.

PX4 HRT follows Gazebo world-clock callbacks in the retained lockstep build.
The sensor's own header carries Gazebo update time, but GZBridge does not
preserve that header/sequence in SensorGps. Arrival/processing order between
world-clock and sensor callbacks has no established ≤500 µs bound. The 4 ms
physics step is not itself a timestamp error bound. Transport latency after a
correct acquisition label would be an availability issue, but latency before
callback-time assignment can affect the label's physical meaning.

A passive Gazebo NavSat header is a candidate acquisition reference. The existing
pre-noise ground-truth diagnostic also exists and was considered; it must never
replace noisy GPS estimator input. Neither diagnostic supplies a preserved,
shared per-event ID with the post-noise GPS observation. Pre-noise coordinates
can repeat in the stationary fixture, post-noise coordinates change, and
capture-scoped uniqueness cannot be assumed before data exist. Joining by
nearest times, by positions after noise, or by callback-time equality would not
independently validate the timing relationship being tested. No approved
nonstationary capture fixture or validated unambiguous association resolves
that gap at this checkpoint. This is a missing qualified association, not a
claim that Gazebo sensor headers or pre-noise diagnostics do not exist.

Therefore the independent acquisition-reference gate is not satisfied before
launch. ADR 0011 requires stopping for a minimal observation design when the
available interfaces cannot identify the events. The F.1 request likewise
requires reporting this limitation before a redundant live experiment. A new
GPS preservation window alone cannot satisfy the primary acquisition objective.
No motion/control experiment, source hook or direct sensor bridge was introduced.

## Task B: frozen qualification protocol

The policy was emitted before any final qualification result by
[m3_gps_qualification.py](../../experiments/python/src/aerotwin_experiments/m3_gps_qualification.py).
The external frozen-preflight JSON SHA-256 is
`8b9b71f045d96d8da7bfe23d09a33ae4ba8e12354eecff2970e62a1206e5b02f`.
Its `independent_reference_available=false` records that no **qualified** event
association is available, rather than denying the existence of candidate
reference topics. No threshold was changed in response to live results.

| Item | Predeclared requirement |
| --- | --- |
| Source / wire | Selected uORB vehicle_gps_position instance 0 / ROS SensorGps topic above |
| Native cadence | Configured 30 Hz, not the estimator's 1 Hz correction interval |
| Count derivation | 30 × 60 = 1,800 nominal events per steady window, 1,801 possible with both endpoints |
| Minimum | At least 1,800 source, wire and unique matched records per session, plus matched/source/wire sample spans ≥60 s |
| Coverage gaps | Maximum consecutive sample gap ≤2 × ceil(10⁹/30) = 66,666,668 ns; report all gaps. This is a coverage policy, not a physical timing bound or completeness proof |
| Identity | Timestamp-independent canonical payload bytes at native precisions, unique on both sides in the same epoch/generation/device context; SHA-256 is an index, actual byte equality is the key |
| Canonical fields | Float64 latitude, longitude, MSL and ellipsoid altitude; float32 N/E/D velocity; device_id, fix_type and satellites_used. Expected simulator 3D fix (3), valid velocity and finite fields required. Signed zero is retained |
| Timestamp acceptance | Exact zero difference for each matched publication and acquisition field, using checked integer µs→ns conversion |
| Integrity | Zero invalid, ambiguous, unmatched or non-forward observations for scoped preservation PASS; reject stale epoch/generation, publisher/device/receiver changes, wrong clock and overflow |
| Startup | Retain startup logs. Exclude a fixed five simulation seconds after verified configuration/receiver readiness before the steady capture window. No synchronization-converged flag is required when synchronization is intentionally disabled |
| Independent reference | Identifiable Gazebo sensor acquisition event linked to its assigned HRT label, with event-generation semantics and observation error accounted for |
| Timing | Total registration uncertainty ≤500 µs; any paired-stream comparison requires ≤1,000 µs worst-case skew. Unknown terms remain unknown; measured maxima alone are insufficient |
| Motion | Stationary plus bounded nonstationary behavior within 5 m/s and 2 m/s², when a suitable declared fixture exists. Missing coverage cannot produce full qualification PASS |
| Attempts | Maximum two in total, independent full-runtime epochs, ≥60 simulation seconds each, ≤180 s wall time per attempt plus ≤30 s owned cleanup |
| Cleanup | Reuse the unique Compose project/run IDs and existing finally/owned-process cleanup. Stop/remove only package-owned services/network. Preserve original build container/image and all raw evidence |

Source capture would use the existing runtime logger configuration mechanism for
`vehicle_gps_position 0` and `sensor_gps 0`, selecting ULog dataset instance 0
explicitly and retaining dropout/sampling metadata. Wire capture would retain
full-precision SensorGps fields, typed publisher discovery, device ID, both
timestamps, reception diagnostics and ingress epoch/generation provenance.
A zero-dropout logger record still cannot prove that every uORB publication was
captured. Any requested/observed parameter discrepancy or additional receiver
requires stopping before data acceptance.

This is a specification and offline schema implementation, not an unexecuted
capture runner represented as validated. The new module does not launch Docker,
install a subscription, parse live ULogs or enable a mapping. It accepts trusted
JSON capture records for preservation checks. Independent acquisition evidence
is deliberately not inferred from those records.

## Tasks C–E: session and uncertainty results

| Planned session | Execution | Simulation span | Source/wire/matched counts | Differences, gaps, registration | Cleanup |
| --- | --- | --- | --- | --- | --- |
| 1 | NOT RUN: offline reference gate blocked | Not measured | Not measured | Not measured | Not exercised |
| 2 | NOT RUN: offline reference gate blocked | Not measured | Not measured | Not measured | Not exercised |

There are no new live GPS samples, exact matches, runtime parameter readbacks or
independent epochs from F.1. Zero attempts does not mean two failed sessions or
zero-dropout capture. No source-to-wire GPS equality or physical registration
PASS is claimed. Prior odometry measurements are not reused as GPS evidence.

Checked integer conversion contributes **0 ns extra rounding** for in-range
values. Positive ns→µs truncation contributes **less than 1 µs** conditional
representation loss. Acquisition-to-callback registration, event association,
source-clock delivery phase and coverage remain unbounded/unmeasured. Addition
of zero in the no-adjustment serializer cannot bound those terms. The total
registration uncertainty is **unknown**, so compliance with 0.5 ms is BLOCKED.
GPS-only data would not independently qualify IMU, pairwise skew or Gazebo truth
alignment even if its timestamp preservation passed.

## Offline implementation and epoch/stale-packet checks

The module validates explicit capture context: epoch, positive mapping
generation, source/wire publisher IDs, nonzero device_id, verified singleton
receiver inventory `[0]`, selected receiver 0 and all four parameter values.
Records retain this provenance individually. They are never silently relabelled
with the current generation. Invalid/ambiguous records are counted and excluded;
ordering is checked before the payload join can conceal arrival reversal.
Unmatched source and wire counts, both differences, spans and gaps are returned.

`compare` can report capture-scoped exact preservation and numerical coverage
for supplied records. It always leaves physical registration, budget compliance
and GPS qualification BLOCKED and `mapping_valid=false`. The caller must verify
ingress/context authenticity; a JSON label cannot detect an old packet that an
incorrect capture layer has deliberately relabelled as new. There is no
production subscription or automatic invalidation wiring in this package.

Focused tests reuse the existing Mapping state machine for initialization,
reconnection invalidation, controlled restart to a new epoch, stale buffered
observations and duplicate rejection. GPS-schema tests additionally reject
publisher, receiver, device, mode and generation changes and backward ordering.
Synthetic fixtures include a native-rate 60 s dataset for deterministic coverage
logic, clearly distinguished from live samples. A synthetic zero uncertainty
used to exercise Mapping is not GPS physical evidence.

## Qualification gates

| Gate | Decision | Scope and reason |
| --- | --- | --- |
| No-adjustment configuration | BLOCKED | Source mechanism confirmed; fresh-session configuration/readback not executed |
| GPS receiver identity | BLOCKED | Candidate topic/instance pinned; actual inventory, selection and device identity not captured |
| Acquisition timestamp semantics | BLOCKED | Callback-HRT assignment confirmed, but required sensor-generation association not independently established |
| Source-to-wire timestamp preservation | BLOCKED | Offline exact-comparison tests pass; no live GPS source/wire captures |
| Independent acquisition registration | BLOCKED | No qualified shared event association through the noise/producer boundary |
| 0.5 ms timing-budget compliance | BLOCKED | Total uncertainty unknown; no thresholds relaxed |
| Observation coverage | BLOCKED | Neither required 60 s session executed; no nonstationary evidence |
| Epoch isolation | PASS, deterministic scope only | Existing Mapping and new ingress-record tests reject wrong epochs/generations. No new live epoch exercise |
| Stale-packet rejection | PASS, deterministic scope only | Old capture provenance rejects; production buffer/ingress wiring is not implemented or qualified |
| Runtime cleanup | BLOCKED, not exercised | No services launched or cleanup required. Preservation of original stopped container/image verified separately |

Overall: **GPS qualification BLOCKED**, with no restricted production M3.4 GPS
adapter scope authorized. The offline work is publishable independently of this
decision and does not complete the M3 integration milestone.

## Verification and reproduction

The existing E.2 host environment and CMake build caches were reused. No new
runtime or plotting dependency was added. Full regression results are recorded
in the external evidence and final PR checks:

| Check | Result |
| --- | --- |
| Complete Python suite | PASS, 149 tests, no skips |
| New GPS qualification cases within that suite | PASS, 30 deterministic cases |
| C++ Debug configure/build and CTest | PASS, 230/230 |
| C++ Release configure/build and CTest | PASS, 230/230 |
| Ruff lint and formatting | PASS |
| Existing-environment dependency consistency | PASS |
| Existing Compose configuration expansion | PASS, no service launch |
| Changed-document local links and Git whitespace | PASS |
| Live GPS preservation/acquisition sessions | NOT RUN, preflight BLOCKED |

Reproduce offline checks with:

```sh
/private/tmp/aerotwin-e2-venv/bin/python -m aerotwin_experiments.m3_gps_qualification \
  --output /tmp/new-gps-preflight.json
/private/tmp/aerotwin-e2-venv/bin/python -m pytest -p no:cacheprovider
cmake --preset dev-debug
cmake --build --preset dev-debug --parallel 2
ctest --preset dev-debug
cmake --preset dev-release
cmake --build --preset dev-release --parallel 2
ctest --preset dev-release
/private/tmp/aerotwin-e2-venv/bin/ruff check .
/private/tmp/aerotwin-e2-venv/bin/ruff format --check .
/private/tmp/aerotwin-e2-venv/bin/python -m pip check
git diff --check
```

For externally acquired records, use `--input capture.json --output new-result.json`.
The input contains `context`, `source` and `wire` arrays as specified in
[focused schema fixtures](../../tests/python/test_m3_gps_qualification.py).
The output must be new. CLI exit code 2 means GPS qualification remains
BLOCKED even when the report is written successfully. Do not treat synthetic
fixtures as real captures.
Existing Compose configuration is expanded with the pinned image and external
paths using `docker compose config --format json`; no services are launched.

Durable evidence is retained outside Git under the Codex visualization directory
`m33f1-gps-evidence`, including the frozen preflight policy, structured blocked
results, verification logs and manifest. There are no F.1 raw live captures.
Generated files, images, credentials and build outputs are excluded from Git.

## One concrete next action

Propose **M3.3-F.2 — GPS acquisition observation-hook design and approval**.
At the existing GZBridge NavSat callback, specify one validation-only atomic
record linking the original Gazebo header time/sequence, callback ordinal,
assigned HRT, device/receiver identity and post-noise SensorGps payload identity.
Preserve normal sensor values and timestamp assignment. Review measurement
perturbation, loss detection, source semantics and run/generation provenance.
This directly addresses the missing association; another median-offset or
transport window does not.

The hook is not implemented here. Any firmware instrumentation/rebuild requires
separate explicit approval under ADR 0011 and the F.1 restrictions. Review the
design before authorizing code or live attempts. If the required information
cannot be observed without changing the measurement path, return the explicit
Option C noisy-sensor architecture decision for approval, rather than silently
substitute perfect ground truth. Do not start F.2 or a new architecture automatically.

## Source references

- [Authoritative ADR and stop gate](../adr/0011-source-time-qualification-and-release-gate.md#decision-and-bounded-next-package)
- [No-adjustment parameter definitions](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/module.yaml#L88)
- [No-sync callback and constructor mode selection](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/uxrce_dds_client.cpp#L69)
- [Client offset initialization](https://github.com/PX4/Micro-XRCE-DDS-Client/blob/711aef423edd1820347b866d1e4164832df35d04/src/c/core/session/session.c#L168)
- [Outgoing subscription/serializer offset](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em#L101)
- [Selected GPS topic declaration](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml#L77)
- [SensorGps message contract](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/SensorGps.msg)
- [GPS selection and publication](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp#L129)
- [Receiver parameters](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_gps_position/params.c)
- [Actual receiver selection/fallback](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_gps_position/gps_blending.cpp#L42)
- [GPS generation/noise/timestamp assignment](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/simulation/gz_bridge/GZBridge.cpp#L713)
- [Native 30 Hz NavSat model](https://github.com/PX4/PX4-gazebo-models/blob/b6127f4ec20de867e215fb5f78ae88b80f371909/models/x500_base/model.sdf#L310)
- [E.3 uncertainty limitations](../architecture/runtime-clock-uncertainty.md), [E.4 release decision](m3-timing-release-gate.md)
