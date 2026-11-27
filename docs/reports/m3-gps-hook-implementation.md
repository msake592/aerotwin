# M3.3-F.3 — Isolated GPS hook implementation and offline verification

Status: **implementation/build/offline package complete for review**.
GPS qualification and **M3.4 production readiness remain BLOCKED**.
Zero live qualification attempts consumed. No SITL, Gazebo simulation, ROS GPS
capture, overhead comparison or qualification session was launched. No PR or
merge was performed. Live acquisition registration is not validated.

**F.4 review correction:** the live-execution review found that Transport
unsubscribe does not fence active or already-selected callbacks. The observer's
shutdown can free state still accessible to a callback. The build/test record
below is retained, but the implementation is **not approved for live use**.
See the [final release gate and R1 finding](m3-gps-acquisition-final-gate.md).

## 1. Recovery and approved scope

Continued on `feature/m3-gps-acquisition-hook-design`, HEAD
`e62966381d939a730d0456bba04e15f59ed5ce56`. GitHub freshly verified PR #30
MERGED at that commit with all four hosted checks SUCCESS. The baseline and
all uncommitted F.2 work were preserved; no redundant branch was created.
The [F.2 design](../architecture/gps-acquisition-observation-hook.md) remains
the authoritative specification. Its file SHA-256 remains
`dea41062c65bdd5277cf13481c593efe6f0df7b4f9883a4da212292e873a4a44`.
The prior [ADR](../adr/0011-source-time-qualification-and-release-gate.md),
[F.1 report](m3-source-time-gps-qualification.md),
[clock uncertainty](../architecture/runtime-clock-uncertainty.md),
[timing requirements](../architecture/estimator-timing-requirements.md) and
roadmap remain the governing constraints.

The F.3 request explicitly approved the five-file GZBridge observer patch,
validation utilities, isolated incremental compilation and offline tests.
No Gazebo source, GPS processing, timestamp assignment, estimator mathematics,
message schema, receiver policy, dependency version or production adapter changed.

## 2. Runtime preflight and installed-library evidence

Docker Desktop was installed but its daemon was initially stopped. Started the
daemon, then inspected the immutable snapshot through an owned read-only,
network-isolated preflight container. The original build container was never
started. Its unchanged stopped state is exit 137, OOMKilled=false; no cause is
inferred from that exit code.

| Identity | Verified value |
| --- | --- |
| Original container | `aerotwin-m322-build`, ID `9a2952433991e12ce0641aa2fc8a7d748e1b95546342bbc6595f6cbf2e7bc369` |
| Base runtime snapshot | `sha256:7212db2bb758fb941de919bbc5077907de0f773c94bcd415d703383031bf93ef` |
| Original PX4 executable SHA-256 | `babe228be6179e8f7abf8af0912ee6b6fae701e9e18cbacfb4d092670ac74402` |
| PX4 HEAD | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, clean before patch |
| Gazebo models submodule | `b6127f4ec20de867e215fb5f78ae88b80f371909` |
| XRCE client submodule | `711aef423edd1820347b866d1e4164832df35d04` |
| Cache | `/work/PX4-Autopilot/build/px4_sitl_default`, 503 MiB, existing Ninja/objects/archives/executable |
| Build platform | Linux ARM64, GCC, GNU++17 GZBridge, RelWithDebInfo, lockstep enabled |

The complete recursive submodule inventory was recorded before modification;
no missing, changed or conflicted recursive submodule entries were reported.
No fetch, upgrade or dependency installation was needed in Docker.

Installed ARM64 packages are `libgz-sensors8` / NavSat **8.2.2-1~noble**,
`libgz-sim8` / plugins **8.15.0-1~noble**, Transport **13.6.0-1~noble**,
msgs **10.4.0-1~noble**. Package metadata identifies sources `gz-sensors8`,
`gz-sim8` and `gz-transport13`; retained apt indexes identify the OSRF repository.
`dpkg -V` for the NavSat/core sensor, simulation plugin and Transport packages
reported no integrity differences.

| Installed library | SHA-256 |
| --- | --- |
| `libgz-sensors8-navsat.so.8.2.2` | `946edd912f94387985c4be684de8d3090e017682a74d549f4f29f35879a8ec47` |
| `libgz-sensors8.so.8.2.2` | `ff48bd63321be7a5ce3bfce5c8a121804c8ed38c6757626ac1cf3ffdb3a01f1f` |
| `libgz-sim8-navsat-system.so.8.15.0` | `f7d87c406f6d7009c190f90b9774d96b052cafb41f814f6e1fc1d7ce951e3bb2` |

This was not limited to upstream source inspection: installed ARM64 disassembly
of NavSatSensor::Update shows the supplied update-duration argument going to
`durationToSecNsec`, the result copied into the message time, then calls to
`Sensor::AddSequence` and `Node::Publisher::Publish`, in that order. Installed
Sensor::AddSequence disassembly was retained too. This confirms availability
of the event stamp/sequence at the installed binary boundary; it is not a
physical registration measurement or a bit-reproducible rebuild of Gazebo.
The version-matched Gazebo commits remain those in F.2.

Transport 13.6.0 source was inspected at
`35fef7ea10c2e133498e9010bd2e397a0ce0c3bf`: `NodeShared.cc` has both received-data
and local PublishThread callback paths. No single-producer assumption is made.
F.3 incorrectly inferred callback quiescence from shared receive/unsubscribe
locking. F.4 confirmed that receive dispatch releases the mutex before calling
copied handlers, so unsubscribe also fails to fence separate-process callbacks.
The MPSC test does not validate Transport teardown; see the correction above.

Docker VM reported 12 CPUs, 8,319,172,608 bytes RAM, about 7.2 GiB available and
402 GiB free overlay space. Host F.2 inspection had 111 GiB available. The build
clone was limited to two CPUs / 6 GiB RAM and had no network or shared mounts.
These are preflight/resource observations, not runtime timing guarantees.

## 3. Exact patch and observation behavior

The reproducible [firmware patch](../../integrations/px4/patches/0001-gps-acquisition-observer.patch)
changes only:

- `src/modules/simulation/gz_bridge/GZBridge.cpp` and `GZBridge.hpp`.
- That directory's `CMakeLists.txt`.
- New `GpsAcquisitionObserver.hpp` and `.cpp` in the same directory.

The complete isolated firmware diff equals the exported patch. It applies
cleanly to an unmodified copy of the pinned upstream files with
`git apply --check --whitespace=error-all`. No full PX4 tree is vendored.
The [patch README](../../integrations/px4/README.md) documents paths, commands,
wire offsets, configuration and restoration. `.gitattributes` disables outer
whitespace interpretation only for stored unified diffs, whose context retains
upstream tabs/blank lines; the applied firmware contents are separately checked.

The callback's original HRT read remains its first statement. An observation
ticket follows it; initialization-only return is recorded. The existing noise
function executes exactly once and the actual completed SensorGps is copied
without arithmetic reconstruction. The enabled branch captures the return of
the existing single publish call, then queries the actual receiver instance
only on success, avoiding get_instance's advertisement side effect after a
failed publication. Disabled compilation retains the original publish call.
Truth diagnostics remain separate and never become estimator input.

The default-OFF CMake option removes observer code/member/thread when absent.
The instrumented build includes it but remains runtime-disabled unless the
explicit enable value is 1. Invalid requested configuration fails before GPS
subscription; later capture failures invalidate evidence while GPS continues.

The fixed 512-byte context binds run/epoch/generation, sensor topic/frame,
upstream and patch/binary/base/config/Gazebo digests. Every 192-byte record binds
its context SHA-256 and records original sensor sequence/time, unchanged HRT,
actual sample/publication labels, receiver/device, native 50-byte canonical
noisy payload, ordinals, outcome and optional host diagnostics. The 128-byte
footer binds counters, timing maxima and a CRC over records/footer. Types,
offsets and validity conditions are in the README. No floating timestamp join
or nearest-time matching is used. Positive ns→µs quantization and checked µs→ns
conversion remain distinct from physical acquisition error.

## 4. Buffering, shutdown and overhead

The preallocated 8,192-slot MPSC queue uses per-slot generations, acquire/release
publication and one reservation CAS attempt. Full or contended reservation is
an explicit loss. There are no callback retry loops, blocking locks, allocation,
hashes, file I/O or extra noise draws. Slot records are 192 bytes, below F.2's
256-byte ceiling. Producer and consumer ordinals, immutable copies and terminal
counts expose loss/order failures.

The deferred owned writer outputs only to a preopened nonblocking FIFO, with
writes no larger than PIPE_BUF. An external owned Python sink handles regular
filesystem I/O. This concretizes F.2's deferred-output design so a stalled
filesystem does not require unsafe cancellation/detachment of a PX4 thread.
SIGPIPE is blocked only in the writer thread. Malformed context, disconnected
reader, write errors, queue overflow, missing footer or incomplete drain cannot
be a clean capture. After subscription quiescence the writer attempts drainage
for up to two host seconds and joins; failure invalidates capture. Linux thread
scheduling supplies no formal hard wall-time guarantee.

`AEROTWIN_GPS_TIMING=1` records host monotonic brackets and conservative maxima
for queue operation and callback work observed after the original HRT read.
Diagnostic atomic contention saturates maxima instead of understating them;
queue high-water can conservatively saturate at capacity. These are diagnostic
upper estimates, not a live observer-bias bound. Probes, copy/commit, atomics,
cache pressure, writer scheduling and sink traffic all have potential cost.
No live overhead numbers are claimed. F.4 describes CPU/RSS/cadence/distribution
measurement, timer/probe cost and the separate approval for additional A/B runs.

## 5. AeroTwin utilities and offline verification

[m3_gps_hook.py](../../experiments/python/src/aerotwin_experiments/m3_gps_hook.py)
reuses existing integer-clock conversion, canonical GPS payload, context and
source/wire preservation checks. It provides strict binary/context/footer/CRC
ingestion, exclusive-output context generation and FIFO sink, raw/selected/wire
association, duplicate/ambiguity/missing/loss/receiver checks, run/epoch/build
isolation, ordering and explicit uncertainty components. Identity is established
before timestamp equality checks. Payload repetition fails rather than being
resolved by timestamp proximity.

The assessor checks all captured events before scoring an explicitly frozen
steady interval, starting at least five simulation seconds after declared
readiness. Startup labels are not silently used for steady coverage. It always
leaves physical registration, GPS qualification and M3.4 BLOCKED; supplied zero
uncertainty and perfect synthetic timestamps cannot enable a mapping. It accepts
trusted capture metadata, not self-authenticating JSON. Publisher discovery,
parameter readback and actual ROS/ULog acquisition are external prerequisites;
no automatic live acquisition/export runner is claimed as tested here.

[Python tests](../../tests/python/test_m3_gps_hook.py) add **47 synthetic cases**:
strict decode/context/CRC, deterministic output, actual-precision identities,
signed zero, duplicate event/payload, ambiguous/missing stages, receiver,
publisher, topic, run/epoch/generation/build mismatch, stale/order/gap checks,
zero/negative/overflowing times, unknown/overflowing uncertainty, frozen steady
coverage, exclusive writes, complete/truncated FIFO sink closure and patch/compile-gate scope.

[Native test](../../integrations/px4/tests/observer_test.cpp) and
[driver](../../integrations/px4/tests/run_observer_tests.py) pass **12 offline
scenarios** against actual generated SensorGps and installed protobuf types:
disabled observer; enabled actual output association and byte/timestamp
preservation; full/wrapping queue and four producers; invalid enable/config/
timing/digest; malformed overflowing sequence; overflow/loss accounting;
shutdown/drain; stalled/disconnected sink; optional timing probes. Tests compile
with PX4's floating-point flags. Signed zero is injected by object representation
so the test does not depend on compiler preservation of a negative-zero literal.
No simulated sensor generation, SITL or live qualification is involved.

## 6. Incremental build and independent runtime identity

Pre-patch dry planning needed only version metadata and relink. Copying files
initially changed mtimes of byte-identical YAML/source inputs, producing a
spurious 402-action dry plan. No such build ran. Restoring original metadata
**only for byte-identical upstream files** reduced it to 21 planned actions.
Modified code was not backdated. Remaining metadata-driven logger/MAVLink/main
recompilation was cache-managed; no clean/full rebuild was performed.

Exact configure/target commands, in the clone at `/work/PX4-Autopilot`:

```sh
export PATH=/work/venv/bin:$PATH
cmake -S . -B build/px4_sitl_default -DAEROTWIN_GPS_OBSERVER=ON
ninja -C build/px4_sitl_default -n modules__simulation__gz_bridge px4
cmake --build build/px4_sitl_default --target modules__simulation__gz_bridge --parallel 2
cmake --build build/px4_sitl_default --target px4 --parallel 2
```

A Python subprocess wrapper records durable checkpoints, command exit status,
monotonic duration and `resource.getrusage(RUSAGE_CHILDREN)` because `/usr/bin/time`
is absent. The unavailable-time wrapper exited before compilation. Initial
module/relink elapsed times were **5.672 s / 3.205 s**; child maximum RSS was
**762,052 KiB**. After adding final tested timing/footer integrity fields, the
reviewed three-action incremental update took **0.559 s / 0.661 s**, with
cumulative child maxima **217,172 / 422,448 KiB**. Both builds passed. About
**10.10 s** total target-command time excludes configure, tests and snapshot
creation. Child maximum RSS is not the aggregate simultaneous VM memory peak.
No full rebuild, dependency upgrade or simulator launch target was used.

| Final review identity | Value |
| --- | --- |
| Firmware patch SHA-256 | `d6768d4b549d302162923b87bbed8ffd2b93d76f3c437bc67368087de21acb13` |
| Instrumented executable SHA-256 | `e135f23733977f56b07b3c63bd1686897d6db8bada09fe483c6b0fb0f1c3f35a` |
| Review snapshot | `sha256:d66443dc476b2626fec1d699b7a5e5f3396dd4d1786dd69b69e6dff2f17c45aa` |
| Local convenience tag | `aerotwin/m33f3-gps-observer:d6768d4b549d` (use immutable digest for runs) |
| Snapshot size | 11,059,020,380 bytes total, including shared base layers; not unique added disk usage |
| Default configuration | Observer compiled ON; image environment explicitly `AEROTWIN_GPS_OBSERVER=0` |
| Owned build container | `aerotwin-f3-build`, independently writable layer, now stopped |

The snapshot labels record phase, upstream, patch and binary digests. No
instrumented artifact replaced or retagged the original reference. Original
container Id/Image/State/Config/HostConfig/Mounts compare unchanged before/after;
its executable was copied read-only and independently hashed again after build,
still matching the original SHA-256. Both owned preflight/build containers are
stopped; no Docker containers remain running at cleanup inspection.

Restore by selecting the original immutable image and original launch manifest,
verifying its executable hash, and stopping only owned instrumented services.
No reverse patch or binary overwrite on the reference is needed. Raw evidence,
source inspections, submodule/package inventories, dry plans, compile/test logs,
build resource checkpoints and image/container manifests are outside Git under
`/private/tmp/aerotwin-f3-evidence`. The reviewed patch/schema/test sources are
in Git's working tree; binaries/caches/raw captures are excluded. Temporary
evidence requires archival before any later host cleanup.

## 7. Regression and scope verification

| Requested check | Actual result |
| --- | --- |
| Full Python suite | PASS, **196 tests**, no skips; 149 baseline + 47 new cases |
| C++ Debug build / CTest | PASS, **230/230**; production core unchanged |
| C++ Release build / CTest | PASS, **230/230**; production core unchanged |
| Focused native instrumentation | PASS, **12 offline scenarios** |
| Isolated PX4 module / executable | PASS, incremental only, final binary recorded above |
| Ruff lint / format | PASS, **79 Python files** formatted |
| Existing-environment dependency consistency | PASS, no broken requirements; no new dependency |
| Docker Compose configuration | PASS, three existing services expanded; none launched |
| Document local links / anchors / structure | PASS, 6 documents, 79 local links, 9 anchors; no broken links |
| Whitespace | Git and explicit new-file checks; firmware patch apply check with error-all |
| Live acquisition / overhead / qualification | NOT RUN, outside F.3 approval; zero attempts consumed |

Native firmware tests are Linux ARM64 only. No new Linux AMD64 firmware build,
Gazebo package rebuild, live ROS DDS capture, installed sensor runtime sample,
clock reset/reconnect exercise, noise trajectory invariance campaign or formal
scheduler worst-case proof was executed. Offline invariance evidence consists
of an additive guarded callback diff and actual-output observer tests, not a
live before/after GPS trajectory comparison.

## 8. F.4 gate, limitations and decisions

The [prepared F.4 protocol](../guides/m3-gps-hook-qualification.md) retains two
independent ≥60-simulation-second sessions, ≥1,800 unique matches each, native
configured 30 Hz, zero ambiguity/unmatched/loss, ≤500 µs registration allocation,
≤1 ms pairwise where independently applicable and the two-attempt maximum.
It specifies ingress/source provenance, event identity, epoch isolation, loss,
steady coverage, integer error calculation, uncertainty, overhead, cleanup and
stop conditions. It does not launch anything or mark the mapping valid.

Remaining limitations: installed-package identity is not a reproducible Gazebo
source build; PostUpdate/ECM event-state semantics need runtime-scope review;
wire/source acquisition/export provenance and actual parameters remain unmeasured;
Transport teardown is restricted to separate-process publishing; finite payload
uniqueness must be observed; random seeds do not control thread scheduling;
FIFO loss invalidates capture; overhead and clock registration are unmeasured;
a fixed capture maximum does not prove universal timing compliance. Unknown
uncertainty terms remain unknown, and the provisional budget is unchanged.

Required decisions: review this patch/build/offline package, then explicitly
authorize F.4's frozen fixture, run-specific capture/Compose manifest and ingress
provenance. Separately decide whether to authorize the extra enabled/disabled
overhead pair (not included in the two qualification attempts). Missing
provenance or materially broader capture work must return for review. No further
firmware change, live attempt, PR, merge, direct sensor path or M3.4 adapter
starts automatically. F.3 is available for implementation review; GPS and M3.4
remain **BLOCKED**.
