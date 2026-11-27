# M3.3-F.2 — GPS acquisition observation hook design

Status: **design package complete; implementation awaits approval**.
GPS qualification and **M3.4 production readiness remain BLOCKED**. No firmware
patch, build, live capture, new adapter or dependency is included. Zero live
qualification attempts have been consumed; the maximum remains two.

## 1. Baseline, authority and evidence

[PR #30](https://github.com/msake592/aerotwin/pull/30) was freshly verified MERGED
on 2026-11-27 (Europe/Istanbul); GitHub records merge time
2026-11-26T23:23:31Z. Its merge commit and fetched `origin/main` are
`e62966381d939a730d0456bba04e15f59ed5ce56`. All four hosted checks completed
SUCCESS: C++ Debug on Ubuntu 24.04 and macOS 15, Python 3.11 and 3.13 on Ubuntu
24.04 ([CI run](https://github.com/msake592/aerotwin/actions/runs/45908999210)).
The working tree was clean before creating
`feature/m3-gps-acquisition-hook-design` directly from that commit.

Authoritative requirements are [ADR 0011](../adr/0011-source-time-qualification-and-release-gate.md),
[F.1](../reports/m3-source-time-gps-qualification.md),
[E.3](runtime-clock-uncertainty.md), [E.1](estimator-timing-requirements.md),
[E.4](../reports/m3-timing-release-gate.md), [roadmap](../roadmap/ROADMAP.md)
and [milestones](../roadmap/MILESTONES.md). This design refines the F.1 missing
association; it does not reopen general clock synchronization.

Source-confirmed candidate configuration remains `UXRCE_DDS_SYNCT=0`,
`UXRCE_DDS_SYNCC=0`, `SENS_GPS_MASK=0`, `SENS_GPS_PRIME=0`, singleton raw
receiver 0, selected `/fmu/out/vehicle_gps_position`, `px4_msgs/msg/SensorGps`,
configured NavSat rate 30 Hz. Runtime settings and receiver identity remain
**unmeasured**, not live verified. No GPS preservation or acquisition PASS exists.

### Source identities and inspection limits

| Component | Exact inspected identity / provenance |
| --- | --- |
| PX4 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`; retained source, generated dispatch, compile commands and HRT code |
| Gazebo models | `b6127f4ec20de867e215fb5f78ae88b80f371909`; retained x500_base NavSat 30 Hz, no NavSat noise element; default maximum physics step 0.004 s |
| gz-sim | Version-matched tag `gz-sim8_8.15.0`, resolved upstream commit `446a44335a45b704b4d36dabcc5508ee34eeb3d8` |
| gz-sensors | Retained installed header reports 8.2.2; version-matched tag `gz-sensors8_8.2.2`, resolved commit `9348f9fe8a11b9d50381819f51e17e136aedab8a` |
| XRCE client | `711aef423edd1820347b866d1e4164832df35d04` (2.4.0) |
| XRCE Agent | `73622810d984349b80bbac0ef55fc0b694d62222` (2.4.3) |
| ROS / messages | Jazzy; px4_msgs `86d8239e962f6939e05c3737784f60c02fa884db` |

The Gazebo commits are immutable **version-matched upstream source**, not a
fresh attestation that installed shared libraries were built byte-for-byte from
those commits. Package/library identity and local patches must be checked in
the implementation preflight. Source inspection cannot certify runtime behavior. Retained compile flags identify
Transport ABI 13 and msgs ABI 10, but do not pin their exact package patch
versions or ROS executor/DDS library commits. The table traces their public
handoff boundaries; internal queue/thread implementation is not freshly
source-attested. F.3 preflight must inventory these exact installed packages
and inspect the matching Transport callback dispatch before selecting queue
concurrency. No ordering or latency guarantee is inferred from an ABI number.

Read-only Docker inspection failed because the daemon socket was absent. The
original `aerotwin-m322-build` container and snapshot were not started, changed
or removed. Their current existence/state and cache sizes could not be freshly
verified. Prior evidence records image
`sha256:7212db2bb758fb941de919bbc5077907de0f773c94bcd415d703383031bf93ef`.
The retained copied PX4 executable was freshly hashed and still matches
`babe228be6179e8f7abf8af0912ee6b6fae701e9e18cbacfb4d092670ac74402`.
No claim is made that hashing the copy re-verifies the inaccessible container.

## 2. Exact measurement path

Paths below are relative to the component repositories at the revisions above.
Source links in section 11 make each upstream boundary reviewable.

| Stage and exact file / function | Time and identity | Transformation and execution boundary |
| --- | --- | --- |
| gz-sim `src/systems/navsat/NavSat.cc`, `NavSat::PostUpdate` | `_info.simTime`, simulation duration; sensor entity/topic | In unpaused PostUpdate, checks due time/connections, calls `Implementation::Update`, then `sensors::Sensor::Update(simTime, false)`. Simulation system execution precedes transport; not a transport receive clock |
| Same file, `NavSat::Implementation::Update` | ECM state for this simulation update | Gets geodetic position via `sphericalCoordinates(entity, ecm)` and world linear velocity; sets sensor latitude/longitude/altitude and ENU velocity. Invalid spherical coordinates can leave stale data: require valid world configuration and no such warnings |
| gz-sensors `src/Sensor.cc`, `Sensor::Update(now, force)` | Passed simulation duration; `NextDataUpdateTime` is scheduling state | Active/rate/trigger gates; invokes virtual `Update(now)`. Rate period is duration-cast to milliseconds (30 Hz becomes 33 ms), then catches up to now. Configured 30 Hz is not an exact measured 33,333,333 ns cadence |
| gz-sensors `src/NavSatSensor.cc`, `NavSatSensor::Update` | Header stamp = `msgs::Convert(now)`, integer seconds/nanoseconds, simulation clock; `frame_id` | Builds NavSat, optionally applies configured noise, normalizes angles, sets geodetic degrees/metres and ENU velocity m/s. Retained model has no NavSat noise; noisy PX4 GPS is generated later |
| Same function and `Sensor::AddSequence` in `src/Sensor.cc` | Header `data` key `seq`, decimal uint64 string, starts 0 and increments per sensor stream | Adds sequence immediately before `pub.Publish(msg)`. This is the original sensor-event identity, scoped by sensor instance and epoch; not a global sequence or header timestamp |
| Gazebo Transport `Node::Publisher::Publish` → PX4 `GZBridge::init` subscription, `src/modules/simulation/gz_bridge/GZBridge.cpp` | NavSat header survives to callback; topic binds model/sensor | Transport thread handoff, queues and possible loss; no source-proven cross-topic order with world clock. No finite delivery/reordering bound assumed |
| Same file, `GZBridge::navSatCallback`, entry at line 713 | First statement reads HRT into `timestamp`, uint64 µs; full original NavSat still available | First callback initializes `_pos_ref` / `_alt_ref` and returns **without SensorGps**. Later callbacks convert ENU up to NED down and narrow velocities to float32. Pre-noise global-position diagnostic is separate truth, not estimator GPS |
| Same file, `GZBridge::addGpsNoise`, called by NavSat callback | Same callback event and earlier HRT reading | Stateful position/velocity noise and random draws change payload. Never use pre-noise coordinates to match post-noise GPS. Observer must not call RNG, re-run conversion or reseed |
| Same callback, SensorGps construction and `_sensor_gps_pub.publish(sensor_gps)` at line 799 | Both `timestamp` and `timestamp_sample` = entry HRT, µs; UTC field 0; generated simulation device ID | No original Gazebo header/seq in SensorGps. Geodetic float64, NED float32 velocities, quality constants, satellite/fix policy. `timestamp` is a producer label, not a new HRT reading at publish |
| `platforms/common/uORB/PublicationMulti.hpp`, `PublicationMulti::publish`; uORB `DeviceNode::write` | Raw instance allocated on advertisement, not necessarily 0; publication generation internal | Synchronous copy into uORB, consumer notifications; latest-value queues can overwrite intermediate samples. Boolean publish return is observable without changing publication call count |
| `src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp`, `Run`; `gps_blending.cpp`, `GpsBlending::update` | Subscribes raw instances; selected SensorGps fields retained without blending | Work-queue boundary, may coalesce updates. Mask 0 / prime 0 still needs singleton verification; fallback or blended output invalidates this contract. Publishes `vehicle_gps_position` |
| `src/modules/uxrce_dds_client/dds_topics.h.em`, `SendTopicsSubs::init/update`; generated `ucdr_serialize_sensor_gps` | `orb_subscribe` selected topic instance 0; reads `session.time_offset/1000` once per update | Client task polls/copies latest uORB; generated GPS subscription interval is 20 ms (50 Hz cap), not a guarantee of 30 Hz delivery. Generated `build/px4_sitl_default/uORB/ucdr/sensor_gps.h` serializer adds offset µs to both timestamps; other candidate identity fields preserved. No event seq serialized. No-sync mode expects exact zero offset, pending live verification |
| Agent `src/cpp/processor/Processor.cpp`, `Processor::process_write_data_submessage`; `src/cpp/middleware/fastdds/FastDDSMiddleware.cpp`, `FastDDSMiddleware::write_data`; ROS `rclpy` subscription callback proposed for F.3 | Same SensorGps fields; DDS publisher identity; separate ROS host monotonic reception ns | XRCE/Agent/DDS/executor boundaries can buffer/drop. ROS receive time is availability diagnostics only. Existing AeroTwin `m3_container.observe` subscribes **VehicleOdometry**; SensorGps capture must be implemented and tested, not presumed available |

The acquisition event defined here is the simulation state sampled by the NavSat
system during PostUpdate at `simTime`, not message construction wall time or the
ideal scheduled next-update instant. `now` comes from that same PostUpdate after
sensor state is populated; the stamp is therefore a candidate original event
time with explicit source semantics. This is a discrete simulator observation,
not an analog GPS integration window. Validate actual installed-system order,
world geodesy, sensor extrinsics and absence of stale ECM data before accepting
this interpretation. A 4 ms physics step neither adds a mandatory 4 ms error
nor proves 0.5 ms accuracy. If that state/event relationship cannot be attested,
the semantics uncertainty stays unknown and qualification stops.

PX4 `GZBridge::clockCallback` independently takes `msg.sim()` sec/ns and calls
`px4_clock_settime`; after its initial realtime attempt it sets monotonic HRT.
The retained compile commands locate the implementation at
`platforms/posix/src/px4/common/drv_hrt.cpp`.
Its `hrt_absolute_time` reads the lockstep scheduler and `px4_clock_settime`
sets it using ns→µs truncation. Compile metadata confirms lockstep enabled.
Clock and NavSat delivery can race: equality of units/origin supplies no bound
on that race. Registration error is precisely what the hook will expose.

## 3. Missing observable and alternatives

The missing observable is one **atomic association** of original NavSat
`(sensor, seq, simulation stamp)` with the *actual* entry HRT and *actual*
post-noise SensorGps bytes handed to uORB. All are available in one invocation
of `GZBridge::navSatCallback`. No Gazebo source patch is initially necessary.
The callback also exposes the initial nonpublishing event; omitting it would
make completeness accounting misleading.

| Approach | Completeness / clocks / identity | Changes, overhead, build, dependencies, reproducibility and maintenance |
| --- | --- | --- |
| A. Existing diagnostics | ULog/listener preserve PX4 labels; passive NavSat has stamp/seq; truth diagnostic is pre-noise. No exact shared identity across noise, so incomplete | No source build; existing tools only. Low instrumentation cost but logger sampling/dropouts remain. Repeated/stationary coordinates and nearest-time joins cannot repair missing identity. Compatible diagnostics, insufficient evidence |
| B. One PX4 conversion hook, recommended | Atomic incoming seq/stamp → existing HRT → actual noisy payload. Simulation/HRT same-origin contract, no host UTC. Unique canonical byte join to selected uORB and ROS, fail on repeats | Small GZBridge-only source patch plus bounded capture helper and host validation. Incremental module compile/relink plausible. Fixed memory, deferred I/O, measurable observer cost. Pin patch/build/schema/seed provenance. Low continuing maintenance; validation middleware stays outside core |
| C. Paired Gazebo/PX4 source hooks | Necessary only if actual incoming header lacks event seq/stamp or update-state semantics cannot be recovered. Both must carry shared original event ID; two uncorrelated logs are insufficient | Additional NavSat generation hook, second build/artifact, queue/loss reconciliation and perturbation. Same simulation clock suffices, no global synchronization needed. More maintenance/dependencies; re-review before expanding patch scope |
| D. Direct Gazebo sensor path | Header/seq survive, but current raw NavSat lacks PX4 noise; does not qualify retained PX4 path | Separately approved architecture, geodesy/frames/noise/covariance/seeds and new external adapter. Broader build/runtime/maintenance cost. Viable fallback if callback-time HRT fails budget or instrumentation is disproportionate; no direct integration here |

B is the smallest intervention that closes the association gap without changing
GPS inputs or time assignment. A passive subscription to the **existing** NavSat
topic may audit sequence endpoints/loss; it is not a second source hook and
must not become an estimator input. If the callback has all required metadata,
C is unjustified. If canonical payloads repeat, stop rather than add a hidden
identity field to production messages.

## 4. Proposed record and exact hook placement

Schema `aerotwin.gps_acquisition.v1`; fixed binary records with explicit encoding
and checked decode. Session header/footer supply repeated context. Never write
raw C++ struct padding or use protobuf serialization order as a canonical key.
All required context must be durably bound by header digest to every record
block. R = required; O = optional diagnostic.

| Field | Type / units / domain | Origin and validity | Need |
| --- | --- | --- | --- |
| schema, build/patch/binary/config digests | uint32 version + fixed SHA-256 bytes, no time | Frozen capture header, exact PX4/image/Gazebo/library/model provenance | R |
| run_id, epoch_id, generation | UUID bytes ×2, uint64 generation, no units | Launcher supplies before startup; full restart gives fresh IDs; immutable for process, generation >0 | R |
| sensor_key | Bounded topic/model/sensor strings + frame ID, no units | Frozen subscription context and incoming frame; one sensor/publisher per isolated partition | R |
| gz_event_seq | uint64, event count | Parse exactly one `header.data["seq"]` value as unsigned decimal; reject absent/multiple/malformed/overflow/wrap | R |
| gz_acquisition_ns | int64 ns, Gazebo simulation | Checked sec×10^9+nsec; sec≥0, 0≤nsec<10^9; header represents defined PostUpdate event | R |
| callback_seq / record_seq | uint64 counts, local epoch | Increment for every callback, including initialization/rejection; record slot ordered independently of writer batching | R |
| assigned_hrt_us | uint64 µs, PX4 HRT | Copy **existing** entry `timestamp`; reject zero/nonrepresentable conversion for accepted GPS | R |
| gps_timestamp_us, gps_timestamp_sample_us | uint64 µs, HRT | Copy actual completed SensorGps, not fresh clock reads; absent for initialization-only record | R when published |
| device_id / raw_instance | uint32 / int32, no units | Actual SensorGps ID and actual PublicationMulti instance after the existing publish; require nonzero ID/instance 0 | R when published |
| payload_identity | 50 fixed bytes, no clock | Existing F.1 canonical `<4d3fIBB`: four float64 geodetic/altitude fields, three float32 NED velocities, uint32 device, uint8 fix/satellites; finite, 3D fix, valid velocity | R when published |
| status / publish_ok | enum / bool | INIT_NO_PUBLISH, PUBLISHED, INVALID_METADATA, CAPTURE_FAILED; actual existing publish boolean, no second call | R |
| callback_count, enqueue_count, drained_count, overflow_count, invalid_count, write_error_count, high_water | uint64 counts, no clock | Out-of-band sticky counters, terminal footer; overflow/wrap/write failure invalidates session | R |
| callback/publish/copy monotonic brackets | uint64 ns, Linux CLOCK_MONOTONIC | Real host clock bypassing simulated PX4 clock; diagnostic overhead only, validate clock resolution | O; required in overhead build |
| actual_publish_hrt_us | uint64 µs, HRT | Optional extra reading near publish, explicitly not SensorGps.timestamp and not acquisition evidence | O |

Payload identity is native IEEE-754 field **bit patterns** encoded little endian;
copy bits with memcpy before asynchronous serialization. Preserve signed zero;
the retained bridge compiler uses `-fno-signed-zeros`, so avoid float arithmetic
or comparisons that could normalize it. ROS/ULog decoders must retain native
precision. SHA-256 is an offline index only; compare full canonical bytes.
NaN heading is outside the key; validate other SensorGps fields separately.
This key is reliable only after proving one occurrence per side within the
entire run/epoch/device context; timestamp proximity never disambiguates it.

Proposed PX4 files (no modifications applied):

- `src/modules/simulation/gz_bridge/GZBridge.cpp`: retain HRT as the very first
  statement; account for initial return; copy original metadata and completed
  SensorGps locally; capture return of the **same** `_sensor_gps_pub.publish`
  call; query instance only afterwards (get_instance can advertise); enqueue
  one completed observation. Do not move timestamp assignment or noise calls.
- `GZBridge.hpp`: observer ownership and compile-time enable guard; no estimator
  fields or message schema changes.
- New `GpsAcquisitionObserver.hpp/.cpp` in that directory: fixed storage,
  bounded record copy, writer lifecycle, header/footer and sticky error counters.
- Same directory `CMakeLists.txt`: default-OFF observer build option and helper
  sources. No uORB, XRCE, Gazebo sensor or public SensorGps changes.

Conceptual order, **not an applied patch**:

```text
NavSat(seq, sim stamp) -> existing entry HRT -> existing init/noise/conversion
                                            -> existing uORB publish once
                                            -> completed immutable record
                                                       |
                                               bounded queue -> owned writer
SensorGps raw -> singleton selector -> selected ULog -> XRCE -> ROS SensorGps
                         \________ offline exact canonical-byte join _______/
```

## 5. Correctness, capture and observer effect

Bound metadata parsing: at most 16 header entries, one sequence value of at
most 20 decimal digits, and fixed session-declared topic/frame lengths. Reject
oversized headers without unbounded traversal/copy.

Preallocate 8,192 slots, each at most 256 bytes (2 MiB ring); freeze maximum
header sizes and handle oversized metadata as invalid, never truncate identity.
At nominal 30 Hz this stores about 273 simulation seconds. That is a sizing
estimate, not a guarantee against arbitrary rates or long wall pauses. The
writer drains on an owned thread; no file operations, logging, allocation,
blocking locks, retry loops or hashes in the sensor callback. One bounded
reserve/copy/commit operation uses lock-free atomics verified on Linux ARM64.
If producer concurrency cannot be excluded from Transport source/runtime
checks, use a bounded multi-producer reservation with explicit ordering and
in-flight counters, not an unjustified SPSC queue or new callback mutex.

Reserve failure increments a sticky loss counter and marks the entire capture
invalid while normal sensor processing continues. A missing/invalid footer,
writer error, partial record, counter wrap or failed checksum likewise fails
closed. No silent record dropping and no blocking to save evidence. File open,
permissions, allocation and header commit must succeed before capture is armed.
Shutdown disarms, waits boundedly for in-flight callbacks, drains writer and
writes counts/checksum; timeout fails capture. No detached writer or dangling
references to `msg`/SensorGps. Uninstrumented compile must remove observer calls,
storage and thread; an instrumented runtime-disable switch must bypass capture.

The observer never mutates `msg`, sensor_gps, RNG state, rate, timestamps or
estimator inputs. Incoming raw coordinates and metadata remain evidence only;
no truth enters estimator APIs. The original executable/image is the reference.
Capturing metadata after the entry HRT cannot retroactively change that reading,
but extra work can affect later clock callbacks, sensor dispatch and CPU load.

Proposed bounded assessment, subject to implementation approval: offline
synthetic replay first (initial return, malformed/duplicate seq, repeated payload,
full ring, stalled writer, short write, disk-full, concurrent producers and
shutdown). Then one predeclared instrumented enabled/disabled comparison pair,
at most 20 simulation seconds per leg, ≤90 wall seconds plus ≤30 cleanup each,
using independent isolated epochs and identical declared world/seed/resources.
These are overhead diagnostics, not qualification attempts or qualification data;
if the approval does not authorize extra runtime sessions, stop after offline
assessment. Declare them before running; never relabel a failed qualification.

Measure host monotonic cost of copy/commit and whole callback, CPU/RSS,
ring high-water, output bytes, callback/event counts and observed simulation
cadence. Proposed review thresholds: measured copy/commit maximum ≤25 µs,
whole-callback added p99 ≤50 µs, RSS increment ≤8 MiB, zero loss/errors and no
new sequence/order anomalies. These are engineering stop thresholds, not
measured results or allocations taken from the 500 µs budget. Compare enabled
and disabled distributions, disclose unpaired scheduling variability, clock
resolution and timer overhead. A finite maximum cannot bound all future
scheduler interference; unknown observer bias remains an explicit uncertainty
term. Do not subtract observed latency to 'correct' acquisition labels.

## 6. Future qualification protocol and uncertainty

Implementation and capture approval are separate from production readiness.
Retain two independent ≥60-simulation-second steady sessions, ≥1,800 accepted
observations **per session**, configured 30 Hz, ≤180 wall seconds per attempt
including startup and ≤30 s owned cleanup. Maximum two attempts total; zero used.
Freeze fixture, world, seed policy, CPU/RAM, hashes, all gates and interval
selection before launch. Verify actual parameters, singleton receiver/device,
selected publisher, no blending/fallback and no-adjustment mode in each fresh
session. Retain startup and exclude a fixed five simulation seconds after
verified readiness. Include declared stationary and bounded motion (≤5 m/s,
≤2 m/s²); no approved motion fixture means no full qualification PASS.

1. Bind run/epoch/generation at ingress, use fresh Compose project, Gazebo
   partition, DDS domain/session and capture files. Flush/quarantine old queues
   before arming; do not attach current epoch labels to queued unknown packets.
   Restart/reset, reconnect, mode/device/publisher change or backward source
   time invalidates acceptance. Pause/resume and in-place reset remain excluded.
2. Parse NavSat seq/stamp at callback; match by `(run, epoch, sensor_key, seq)`.
   Require strict increasing seq, no duplicates, gaps or wraps over the audited
   interval. Initialization has its own nonpublishing record outside steady GPS.
   Use passive existing NavSat capture for pre/post-window sequence anchors to
   audit missing callback endpoints. Missing anchors cannot prove complete
   sensor coverage; no extrapolation from logger dropout counters.
3. Join hook noisy payload bytes → raw sensor_gps instance 0 → selected
   vehicle_gps_position instance 0 → ROS SensorGps within identical provenance.
   Require unique canonical key on every side and full byte equality; report
   raw/selected/wire counts separately. Existing F.1 code checks selected
   source/wire only; multi-stage hook and raw joins need new offline utilities.
   Check timestamps **after** identity matching: selected and ROS fields must
   equal the actual hook fields exactly (zero µs difference) in no-adjustment
   mode. Unmatched, ambiguous, invalid and non-forward records all fail; never
   silently thin the dataset. Do not use timestamp equality as the key.
4. Require all four stages ≥1,800 unique matches; acquisition, assigned-sample,
   selected-source and wire spans each ≥60 s. Include both endpoints when
   needed; 1,800 nominal 30 Hz samples alone can span only 59.9667 s. Keep F.1
   maximum consecutive sample gap ≤66,666,668 ns and report actual intervals.
   Actual millisecond-throttled Gazebo cadence must be measured, not snapped
   to ideal 30 Hz. Callback seq + counters + footer establish hook completeness;
   exact downstream one-to-one sets detect uORB/XRCE/DDS/observer losses. A zero
   ULog dropout count alone is insufficient. Require zero capture losses.
5. Use checked signed integer arithmetic:

   ```text
   a_i = gz_acquisition_ns
   h_i = checked(assigned_hrt_us * 1000)
   e_i = checked(h_i - a_i)
   E_observed = max_i(abs(e_i))
   U_capture = E_observed + U_event + U_mapping + U_observer + U_representation
   ```

   Simulation origin/rate mapping is identity only with confirmed installed
   lockstep/world-clock source and common epoch. Never fit an offset to hide
   callback lag, subtract each stream's first sample, or map via host reception.
   `e_i` directly includes realized clock-delivery phase and callback scheduling
   relative to acquisition. No need to measure every world-clock packet to
   calculate that difference; no separate global synchronization campaign.
   Any nonidentity mapping needs independently evidenced origin/rate residual
   bound over the full horizon; absent evidence stays unknown.
6. `U_event` covers uncertainty in discrete PostUpdate state/event semantics,
   including invalid/stale ECM paths. `U_mapping` covers residual origin/rate
   interpretation, not delivery phase already observed in e_i. Positive
   ns→HRT µs truncation is <1,000 ns; checked µs→ns adds exactly 0 ns. The
   observed difference already includes realized truncation; a conservative
   additional <1,000 ns allowance may be reserved for representation scope,
   explicitly avoiding presenting that as a second measured error. Reject
   zero, overflow, invalid header nanoseconds and unsupported clock changes.
   `U_observer` must have a defensible scoped bound or remain unknown; p99 and
   an A/B observed maximum cannot supply a hard worst-case term. Add bounded
   terms conservatively, no unapproved RSS/statistical independence assumptions.
7. Accept **capture-scoped acquisition evidence** only if identity, completeness,
   semantics, mode, motion, ordering and both-session coverage all pass, and a
   justified total bound is ≤500,000 ns. Any observed |e_i| >500,000 ns is an
   immediate failure before adding other terms. Unknown uncertainty prevents
   budget PASS even when every observed difference is small. Pairwise ≤1,000,000
   ns requires two independently bounded streams (conservative sum); GPS-only
   F.3 cannot qualify IMU or ground-truth pairwise alignment.
8. Report signed min/max, absolute max, distributions, count/span/gap and
   rejection tables, uncertainties and scope separately. A measured maximum
   bounds captured events conditional on reference validity; it does not prove
   a universal future bound, tail probability or operational deadline. Production
   readiness needs a reviewed operating envelope and coverage/confidence or
   formal scheduling bound beyond finite observation. Do not mark M3.4 ready
   merely because two windows pass.
9. Disarm/drain capture, preserve raw files, headers/footers, inventory/readbacks,
   checksums and structured failures. Stop/remove only package-owned services
   and network in finally cleanup; verify no owned processes remain. Retain
   original container/image and immutable reference binary. No third attempt.

## 7. Build feasibility and isolation plan

Retained `compile_commands.json` identifies Linux ARM64 GCC, GZBridge GNU++17,
`-O2 -g`, lockstep, gz-transport13, msgs10, math7, and build root
`/work/PX4-Autopilot/build/px4_sitl_default`. The inspected module CMake target
is `modules__simulation__gz_bridge`; executable target is `px4`. Incremental
module compile and final relink are **feasible in principle**, not freshly
verified against an accessible cache. No build was executed.

After approval, inspect Docker read-only first: image/container identity,
source HEAD/submodules and dirty state, binary hash, cache toolchain/paths,
package/library versions, free disk and VM RAM. Stop on missing/mismatched
artifacts; no dependency installation or silent full rebuild fallback.
Create a new container from the immutable snapshot with an independent writable
layer, no mounts onto original source/build/runtime volumes. Preserve identical
`/work/PX4-Autopilot` absolute paths so copied cache artifacts remain useful;
copy any externally mounted build/source volume into package-owned storage.
The old container and image remain untouched.

Proposed build commands **for later approval only**, inside the clone:

```sh
cmake --build /work/PX4-Autopilot/build/px4_sitl_default \
  --target modules__simulation__gz_bridge --parallel 2
cmake --build /work/PX4-Autopilot/build/px4_sitl_default \
  --target px4 --parallel 2
```

Enable the default-OFF observer option through the existing CMake cache in the
clone before those commands; record exact configure invocation/cache diff.
First use a dry dependency plan after approval to check rebuild scope. Never
invoke `gz_x500` as a build target: its custom target **runs the simulator**.
Expect recompilation of GZBridge/helper and consumers of its header, module
archive and executable relink. Generated headers, unrelated modules, Agent,
ROS overlay, Gazebo and model assets should be reusable if cache-compatible;
CMake regeneration or stale dependencies can broaden this and require review.

Planning estimate, not a benchmark: 3–5 engineering days for hook/lifecycle,
offline joins/fault tests, patch review and isolated build preparation; another
1–2 days for approved overhead/capture review. Warm incremental compile/relink
may take 5–30 minutes on two ARM64 CPUs, but is unmeasured; reassess if the dry
plan expands. Use two CPUs / 6 GiB build RAM as the retained build-guide starting
limit, distinct from the existing 2 GiB runtime service limit. Plan 10–20 GiB
additional disk headroom for writable clone/cache and evidence; actual image,
cache and VM space must be measured before cloning. Host inspection showed
111 GiB available; Docker VM disk/RAM are unavailable, so resource sufficiency
is not established. No NVIDIA hardware is required.

Reproduction artifact: clean pinned upstream HEAD/submodule inventory,
reviewable git-format patch and digest, schema/tests, compiler/package manifest,
CMake cache/options, dry plan/build log, enabled/disabled binaries and SHA-256,
new image digest and isolated launch manifest. Proposed firmware change is
approximately three edited files plus two helper files, 250–450 C++/CMake lines;
AeroTwin capture/decoder/join and tests roughly 300–600 Python lines. These are
scope estimates, not applied code or permission to expand into a bridge.

Restore by stopping only package-owned processes/containers, selecting the
original immutable image and original launch/configuration manifest, and
verifying its binary hash. Do not copy the instrumented binary over the old
one, retag the reference image, or 'restore' it by reverse-patching the original.
Keep evidence and patch independently even if the disposable clone is removed.

## 8. Risks, decision and explicit approval gate

Principal risks: installed Gazebo source mismatch; stale ECM state; sensor-local
sequence restart/wrap; callback concurrency; observer perturbation; payload-key
collisions; sampled downstream losses; unavailable build cache; random-seed
reproduction; unsupported reset/reconnect; no approved motion fixture. Record
PX4/Gazebo seeds and RNG algorithm/state provenance where available; the hook
must not change seeding. Identical seeds alone do not guarantee identical
thread scheduling or repeated live noise trajectories.

Recommend **B**, one validation-only hook at NavSat conversion, with optional
passive existing-topic sequence audit. It measures the missing association;
it does not repair timestamp semantics if callback HRT fails the allocation.
On failure, report it and return ADR 0011's separately approved noisy direct
sensor architecture alternative. No automatic timestamp reassignment,
additional hooks, IMU work or production integration follows.

Exact next proposed package: **M3.3-F.3 — Isolated GPS acquisition-hook
implementation and offline verification**. User approval must explicitly cover:

1. The listed PX4 files, default-OFF observer schema/queue/writer, independent
   clone and reproducible patch against the pinned revision.
2. Incremental module build/relink in that clone, resource limits and production
   message/math/noise invariance tests; no original snapshot modifications.
3. AeroTwin validation-only SensorGps capture, hook decoder/multi-stage joins,
   synthetic loss/ambiguity/epoch/stale-packet tests and reference preservation.

F.3 stops after patch/build/offline verification for review unless approval also
explicitly covers the bounded overhead sessions in section 5. The two real
qualification sessions in section 6 need a subsequent explicit execution gate
with verified semantics, perturbation assessment and frozen fixture. No attempt
is consumed by this design. This approval separation follows ADR 0011's explicit
requirement to obtain firmware/source instrumentation approval before a hook;
the present user request also expressly prohibits applying patches or builds.
No PR is opened before design review. M3.4 remains **BLOCKED**.

## 9. Verification scope

This package changes repository documentation only. Applicable checks are
Markdown structure/fences, relative document links and anchors, whitespace,
existing-environment dependency consistency, and Ruff lint/format. No AeroTwin
validation utility changed, so no new utility regression test or C++ rebuild
is needed. The completed timing sensitivity campaign is not repeated.
Verification outcomes are recorded in section 10 after execution.

## 10. Verification results

| Check | Result |
| --- | --- |
| PR #30 merge and hosted CI | MERGED; all four checks SUCCESS; fetched baseline matches merge commit |
| Markdown structure, fences and local links | PASS: 3 changed/new documents, 58 relative file links, 8 anchors checked |
| Existing-environment dependency consistency | PASS: pip check reports no broken requirements |
| Ruff lint / format | PASS; 73 Python files already formatted |
| Git whitespace plus explicit new-document whitespace | PASS |
| Retained reference executable SHA-256 | PASS: matches prior immutable-binary inventory |
| Fresh Docker image/container/cache inspection | NOT VERIFIED: daemon socket absent; no runtime action attempted |
| C++ / Python regression rerun | NOT RUN: documentation-only change, no utilities or implementation modified |
| PX4 builds / live qualification / timing campaign | NOT RUN by scope; zero qualification attempts consumed |

These are local design-package checks, not new hosted CI or runtime qualification.
The temporary checker and downloaded source copies remain outside the repository.

## 11. Pinned implementation references

- [Gazebo NavSat PostUpdate and ECM sampling](https://github.com/gazebosim/gz-sim/blob/446a44335a45b704b4d36dabcc5508ee34eeb3d8/src/systems/navsat/NavSat.cc#L111)
- [NavSat event stamp, payload and publication](https://github.com/gazebosim/gz-sensors/blob/9348f9fe8a11b9d50381819f51e17e136aedab8a/src/NavSatSensor.cc#L155)
- [Sensor scheduling and sequence generation](https://github.com/gazebosim/gz-sensors/blob/9348f9fe8a11b9d50381819f51e17e136aedab8a/src/Sensor.cc#L479)
- [PX4 NavSat callback and noise](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/simulation/gz_bridge/GZBridge.cpp#L678)
- [PX4 clock callback](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/simulation/gz_bridge/GZBridge.cpp#L331)
- [uORB PublicationMulti lifecycle](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/common/uORB/PublicationMulti.hpp#L72)
- [Selected GPS publication](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp#L102)
- [XRCE outgoing dispatch template](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em#L101)
- [Module build targets](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/simulation/gz_bridge/CMakeLists.txt)
- [F.1 canonical identity and blocked qualification utility](../../experiments/python/src/aerotwin_experiments/m3_gps_qualification.py)
- [Current odometry-only runtime observer](../../experiments/python/src/aerotwin_experiments/m3_container.py)
- [Pinned build and runtime recovery guide](../guides/m3-sitl-runtime.md#m322-docker-arm64-results)
