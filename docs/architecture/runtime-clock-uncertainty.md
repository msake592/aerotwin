# M3.3-E.3 runtime clock uncertainty validation

Status: **analysis complete, timing-budget compliance BLOCKED**. Recommend a
restricted, topic-specific source-clock contract for review, with production
inputs disabled until their acquisition and mapping uncertainties are bounded.
This records the E.3 checkpoint before E.4. The subsequent
[E.4 release gate](../reports/m3-timing-release-gate.md) and
[ADR 0011](../adr/0011-source-time-qualification-and-release-gate.md) consolidate
the decision: M3.4 remains BLOCKED. The numerical timing/motion values are
approved for continued prototype experimentation only.

## 1. Recovery, provenance and evidence classes

Continued on `feature/m3-estimator-timing-budget` at
`9699dee7b3070762d8a3010413a2519466ea12cb` (also the locally recorded
`origin/main`). [PR #28](https://github.com/msake592/aerotwin/pull/28) was freshly
verified MERGED at that commit, with four successful hosted checks. All existing
uncommitted E.1/E.2 files were preserved. Their checksums were recorded before
this document was created and checked afterwards. No estimator sweep was rerun.

The Docker daemon was reachable through an authorized read-only inspection.
The immutable local runtime image
`sha256:7212db2bb758fb941de919bbc5077907de0f773c94bcd415d703383031bf93ef`
exists as Linux ARM64. `aerotwin-m322-build` exists, stopped, exit 137,
`OOMKilled=false`. No cause is inferred for that exit. There were no running
containers. No container was started, restarted, created or removed.

Pinned source and generated headers were copied from the stopped container:

| Component | Recovered identity |
| --- | --- |
| PX4 v1.17.0 checkout HEAD | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| Preserved PX4 binary SHA-256 | `babe228be6179e8f7abf8af0912ee6b6fae701e9e18cbacfb4d092670ac74402`, matches the existing inventory |
| XRCE Client submodule HEAD | `711aef423edd1820347b866d1e4164832df35d04`, CMake project version 2.4.0 |
| XRCE Agent v2.4.3 checkout HEAD | `73622810d984349b80bbac0ef55fc0b694d62222` |
| Prior validated stack context | Gazebo 8.15.0, ROS 2 Jazzy, px4_msgs `86d8239e962f6939e05c3737784f60c02fa884db` |

Gazebo/ROS were not executed or re-versioned in E.3. Build metadata confirms
`ENABLE_LOCKSTEP_SCHEDULER` and `__PX4_POSIX` for the retained HRT, GZBridge and
XRCE client compilation. Copying source does not independently certify all
object files against source. The recovered binary hash matches the earlier
validated artifact. This is ARM64 evidence, with no new Ubuntu AMD64 validation.

Evidence labels below mean:

- **Measured:** earlier scoped live observations, not a guaranteed future bound.
- **Source-confirmed:** implementation behavior inspected at the identities above.
- **Analytically bounded:** arithmetic bound with stated preconditions.
- **Unmeasured:** no defensible numerical uncertainty bound presently available.
- **Not applicable:** a relationship eliminated by a declared restricted contract.

The [E.1 allocation](estimator-timing-requirements.md) and
[E.2 sensitivity study](../reports/m3-estimator-timing-sensitivity.md) remain
provisional: 0.5 ms per stream, 1 ms pairwise skew, V≤5 m/s, A≤2 m/s², 1 s GPS
prediction/correction and 10 ms diagnostic IMU cadence. E.2's 5,880 runs and
20 seeds support sensitivity assessment, not physical synchronization accuracy.

## 2. Task A: clock-domain inventory

| Domain | Origin and units | Rate | Pause and reset | Synchronization mechanism | Available observation and estimator relevance |
| --- | --- | --- | --- | --- | --- |
| Gazebo simulation | World simulation origin, seconds/nanoseconds | Simulation seconds, configurable relation to wall seconds | Pause stops simulation progression. Full world restart creates a new origin. In-place reset is unsupported by AeroTwin | World clock messages | Gazebo stats and clock/sensor message headers. Physical event-time reference needed for future simulator truth comparison |
| PX4 HRT | SITL lockstep origin, integer µs | Set from received Gazebo clock values, not a calibrated host oscillator | Holds the last applied clock value during simulation pause. Scheduler permits assigning backward time. Full PX4 restart loses prior clock state | GZBridge clock callback → `px4_clock_settime(CLOCK_MONOTONIC)` → lockstep scheduler | uORB timestamps/listeners/ULog. Candidate source timeline for GPS and diagnostics, subject to callback/event alignment |
| Sensor acquisition labels | Topic-specific sample definition on PX4 HRT, µs | Event-driven, sensor/integration cadence | Inherits HRT discontinuities, plus individual sensor selection and integration state | Producers assign sample fields. Some assign callback-time HRT, others preserve integrated-sample labels | GPS `timestamp_sample`, raw IMU sample stamps, combined IMU relative stamp. A common numerical clock does not prove common physical acquisition semantics |
| PX4 publication labels | Usually HRT at publication, µs | Topic/update cadence | Inherits source clock transitions | Publisher chooses timestamp semantics | Odometry `timestamp` differs from `timestamp_sample`. SensorCombined `timestamp` is gyro sample time, despite its generic field name. Publication time cannot universally substitute for acquisition time |
| XRCE-adjusted wire | Source timestamp plus current session offset, integer µs | Source progression plus offset changes | Source pause does not freeze Agent time. Restart/reconnect can change offset. No continuity guarantee | Timesync callback sets session offset. Serializer adds it at dispatch time | Matched odometry transformation, `timesync_status`, client status. Mapping to source acquisition time needs current, message-associated correction or a validated source-time mode |
| ROS reception monotonic | Linux VM boot/time namespace, ns | Linux monotonic elapsed time | Continues while simulation pauses. New boot/namespace or container identity needs rechecking | OS monotonic clock, no automatic simulation mapping | Observer records `time.monotonic_ns()`. Useful for availability and deadlines. Not an acquisition timestamp |
| Wall/system clocks | Agent/ROS Linux system epoch and separate macOS host system epoch, ns or epoch seconds | System time, subject to adjustments | Continues during simulation pause. Can step/slew independently of simulation | OS clock management, no validated AeroTwin absolute reference | Observer wall capture and system-backed ROS clock. Host wall synchronization is unnecessary for event-time-only estimation, but mandatory if absolute cross-domain comparisons are required |

The first GZBridge clock callback attempts `CLOCK_REALTIME` once and subsequent
callbacks set the PX4 monotonic simulation clock. The code does not establish
that the realtime set succeeded. In an unprivileged container this is not proof
of setting Linux wall time. HRT comes from the lockstep scheduler in this build.
The scheduler setter directly assigns the supplied time, without an epoch guard.
The AeroTwin validation layer supplies the reset policy, not PX4's time type.

Agent `get_epoch_time` uses `std::chrono::high_resolution_clock`. The retained
libstdc++ 13 header aliases that type to `system_clock`. This supports the system-clock interpretation for the retained toolchain,
without certifying external UTC calibration or macOS alignment.
The prior observer has `use_sim_time=false` and separately records Linux wall
and monotonic reception readings. Shared VM clocks require verified boot IDs
and time namespace offsets before comparing container reception clocks.

### Topic-specific acquisition semantics

**GPS:** `GZBridge::navSatCallback` reads `hrt_absolute_time()` on callback entry,
assigns it to both raw `sensor_gps.timestamp` and `timestamp_sample`, and sets
`time_utc_usec=0`. It does not use the incoming NavSat acquisition header for
these labels. The exported `/fmu/out/vehicle_gps_position` is a SensorGps type,
forwarded through the GPS selection/blending module, rather than a direct export
of the raw sensor_gps instance. Selection and blending must be pinned: blending
forms weighted timestamps, which cannot automatically represent one original
sensor event. No measured bound relates the callback-time label to the Gazebo
sensor acquisition instant. Source assignment alone cannot certify 0.5 ms.
Renaming callback time as acquisition time would not remove the physical
measurement-to-state association requirement.

**IMU:** `GZBridge::imuCallback` takes one HRT reading for raw gyro and accel
sample/publication fields, ignoring the incoming IMU acquisition header. Later
VehicleIMU integrates readings. Voted SensorCombined carries gyro sample time,
accel relative time and integration durations. Its serializer adjusts the main
timestamp but leaves the relative delta unchanged. The invalid relative-time
sentinel must be rejected before deriving accel time. Integration-window endpoint
and averaging semantics matter for any later fusion. The 10 ms M2 diagnostic
cadence is not a demonstrated live sensor cadence or timing error bound.

**Odometry:** EKF2 assigns `timestamp_sample` from its IMU sample and normally
`timestamp` from HRT at publication. It is a PX4 estimate. It is neither simulator
ground truth nor an independent physical acquisition-time reference.

### Required relationships

| Consumer | Mandatory relationship | Legitimately unnecessary relationship |
| --- | --- | --- |
| GPS-aided CV estimation | Valid GPS acquisition labels, common epoch/origin, calibrated simulation-second intervals and bounded event registration. Exact prior/measurement labels remain mandatory | Host UTC and reception clock alignment, if all inputs and outputs remain event-time source-relative |
| Diagnostic IMU | Preserve declared gyro/accel sample semantics and relative integration timing. Cross-GPS alignment becomes mandatory when comparing their event times | GPS-to-IMU alignment for isolated unpaired diagnostics, absolute host UTC |
| PX4 odometry comparison | Source-time reconstruction for odometry sample time and independent AeroTwin estimate association in that same epoch | Treating publication time as sample time, host wall alignment for same-source comparisons |
| Future Gazebo ground-truth comparison | Independent simulator acquisition/event time to PX4 sample-time registration, frames and physical event semantics | Host wall alignment when truth and estimates share a validated simulation timeline |

## 3. Task B: run-relative feasibility and inversion

Let integer source time be sᵢ and the actual serialization offset be oᵢ, in µs.
For the inspected outgoing timestamp fields the valid-range transformation is

```text
wᵢ = sᵢ + oᵢ
(wᵢ − w₀) − (sᵢ − s₀) = oᵢ − o₀
ŝᵢ = wᵢ − ôᵢ                  reconstruction error = oᵢ − ôᵢ
(wᵃ − wᵇ) − (sᵃ − sᵇ) = oᵃ − oᵇ
```

An exact inverse needs the offset actually applied by the serializer, not an
accurate estimate of absolute UTC. Even an inaccurate synchronization estimate
cancels if the same applied value is removed exactly. Thus synchronization
asymmetry is unnecessary for this source-only inverse. A median or nearest-status
inverse instead needs a defensible bound on offset-association error. Absolute
cross-domain accuracy becomes mandatory only when a consumer actually compares
source events against that other domain.

A common constant offset cancels. A changing offset distorts intervals and
cross-stream registration. Independently subtracting each topic's first packet
also discards the inter-topic origin and cannot establish alignment. Subtract
integer origins before conversion to seconds. Check signed range, source
validity and synchronization mode, including unsigned serializer wrap hazards.

Dispatch reads `session.time_offset/1000` once per update and passes it to each
selected serializer. VehicleOdometry and SensorGps add it to publication and
sample fields. SensorCombined adds it only to the main sample timestamp. A
single message's publication-minus-sample age, or combined accel-relative delta,
therefore cancels that message's common offset. Messages sent in different
updates need not share an offset.

[M3.3-D](../guides/m3-source-wire.md) matched 575 and 565 unique odometry payloads
with no ambiguous captured fingerprints. Offset ranges were respectively 1 µs
and 4 µs, and maximum median-relative residuals 1 µs and 3 µs. Therefore interval
distortion attributable to the observed offset differences is at most 1/4 µs
**among those matched endpoints only**. Publication and acquisition offsets
were identical per pair. The 0/2 µs wire-minus-source span differences over
5.744/5.640 s are compatible with small observed variation, not independent
rate calibration. These results do not validate GPS/IMU acquisition or bound
uncaptured messages, transitions or future operation.

### Initialization, rate and invalidation

The [existing Mapping state machine](../../experiments/python/src/aerotwin_experiments/m3_clock.py)
requires explicit validation, epoch, generation, publisher, domain and semantics.
It rejects stale generations, wrong epochs, invalid ranges and non-forward
observations. It supports reconnect and synchronization-change invalidation.
Those deterministic controls are usable infrastructure. They do not supply
uncertainty evidence or enforce the provisional 0.5 ms allocation by themselves.

A sub-budget offset step can preserve timestamp ordering and publisher identity.
It is not automatically detected by `convert`. No production callback currently
connects every XRCE offset change to `synchronization_changed`. An observation
must be gated on trustworthy mapping-generation metadata, not stamped with the
current generation after arrival. Queued old packets need a verified session
boundary or explicit discard/quarantine before accepting a revalidated stream.

A controlled full Compose restart provides distinct run/epoch IDs and isolated
runtime processes. The validation tests reject observations from the previous
epoch. Reconnection keeps simulation time but invalidates the mapping generation.
Publisher changes, source regression, synchronization changes and mode changes
require fresh evidence. In-place reset remains unsupported. A UUID alone cannot
prove physical source continuity, stale-queue drainage or event-time alignment.

Simulation seconds are the intended estimator time unit, even when real-time
factor differs from one. Wall-clock tracking of simulation rate is unnecessary
for a purely source-relative estimator. What remains mandatory is correct
source intervals relative to physical simulator event times. An HRT value set
by the last received world-clock callback does not bound its lag/lead relative
to an independently delivered sensor header. Pause/reconnect continuity is
excluded from the proposed first operating contract.

## 4. Task C: structured uncertainty budget

Hard bounds may be added only when each term is valid over the stated scope.
No independence assumption or root-sum-square combination is made here.

| Contribution | Classification and evidence | Numerical bound usable now | Budget impact / limitation |
| --- | --- | --- | --- |
| Timestamp quantization | Analytically bounded for positive Gazebo ns→HRT µs truncation | Less than 1 µs representation loss per conversion, conditional on correct source association | Below 500 µs by itself. Does not bound callback acquisition error. Two independently truncated endpoints conservatively contribute less than 2 µs |
| Integer conversion | Analytically bounded, checked host µs×1,000 | Exactly 0 ns extra rounding for valid signed-range integers | Overflow/invalid inputs reject. Session offset is already an integer-µs estimate in this callback path, so dividing its ns representation by 1,000 adds no fractional loss |
| Source acquisition semantics | Source-confirmed callback HRT assignment, GPS selection/blending and IMU integration | Unmeasured physical acquisition registration | Cannot claim per-stream 500 µs or pairwise 1,000 µs compliance |
| Source-to-wire transformation | Source-confirmed addition, measured only for matched odometry | Exact algebra under valid ranges, no absolute physical accuracy bound | Reversal requires actual message-associated offset, not a nearest status row |
| Offset variation | Measured M3.3-D odometry ranges 1/4 µs and median residual maxima 1/3 µs | No general hard bound | Valid observed-endpoint statements only. Does not establish GPS/IMU behavior |
| Clock-rate error | Source-confirmed Gazebo→HRT update and prior odometry span evidence | No independent event-time rate bound or validation horizon | Simulation/wall rate differences are allowed under source-only operation. Event-time clock delivery error remains unbounded |
| Synchronization asymmetry | Unmeasured for Contract A, source-confirmed midpoint assumption. Not applicable to B/C after exact inversion or validated source-time export | No absolute asymmetry bound. Accepted-sample RTT threshold is 10 ms, not 0.5 ms accuracy | Conditional half-RTT envelope can approach 5 ms for absolute alignment. Do not charge this term to a source-only contract that genuinely removes the transform |
| Processing/delivery latency | Source-confirmed callbacks, earlier ordered offline invariance | No live bound on acquisition-to-callback age or application deadline | Post-acquisition delivery latency is not registration error when correct timestamps and ordering are preserved. Callback-time stamping makes pre-label delay relevant |
| Capture completeness | Measured 1,140 matched odometry observations, no logger dropouts, 4–12 ms sampling | No completeness guarantee | 12 ms sampling is not a timestamp error on an identified message. Missed messages prevent all-publication/transition claims |
| Cross-stream acquisition alignment | No independent GPS/IMU/Gazebo matched acquisition capture | Unmeasured | Common HRT type alone cannot establish 1 ms skew |
| Epoch transitions | Source-confirmed reset hazards, deterministic Mapping tests and prior independent run identities | Logical stale-epoch rejection, not numerical clock uncertainty | Supported only through controlled full restart and fresh validation. In-place reset blocked |
| Agent reconnection | Earlier recovery and publisher-change evidence, deterministic invalidation tests | No continuity, stale-packet or post-reconnect uncertainty bound | Invalidate and quarantine. Initial restricted contract requires full restart rather than seamless recovery |
| Host absolute synchronization | Not applicable to the proposed source-only event-time relationship | No UTC bound claimed | Still unmeasured and mandatory if Contract A or absolute host-time comparisons are later required |

For a source reconstruction with known error terms, one could require

```text
Ustream ≤ Uacquisition + Urepresentation + Uoffset-association + Urate(T)
Ustream ≤ 500 µs
UstreamA + UstreamB ≤ 1,000 µs              worst-case pairwise allocation
```

This is a proposed accounting rule, not an evaluated bound. Acquisition and
offset-association terms are unknown, so the sum is **unknown**, never zero.
Under a validated source-time serialization mode the offset-association term
could be eliminated. Acquisition semantics and epoch/rate checks would remain.
A sample's empirical residual cannot replace an unknown term in this sum.

## 5. Task D: synchronization diagnostics and their limits

The retained implementation exposes these existing interfaces:

| Observable | Interface and source | What it can establish | What it cannot establish |
| --- | --- | --- | --- |
| Filtered and raw offset | uORB `timesync_status`: source protocol, remote timestamp, observed_offset, estimated_offset, RTT and HRT publication time | DDS-protocol sample estimates and their evolution. Select source protocol DDS (2) and correct instance | Exact offset attached to each outgoing data sample, acquisition-time error, one-way asymmetry |
| Convergence | `uxrce_dds_client status` and startup log | Filter has accepted at least 500 updates | Sub-500 µs accuracy, stable rate, no future offset steps |
| RTT and remote/local observations | `timesync_status` and underlying XRCE reply implementation | Conditional round-trip diagnostics and remote timestamp for a sample | Exported atomic four-clock exchange, independently calibrated rate during pause, serializer association |
| Offset updates/discontinuities | Startup offset log, status history, debug messages and filtered estimate history | Some observed changes and resets | Complete update sequence: DDS status export is rate-limited to 10 Hz, source logging can also miss updates |
| Rate stability | Source clock/world stats and diagnostic histories | Scoped empirical trend when time domains are properly identified | Independent physical event alignment or guaranteed future drift bound |

The PX4 Timesync filter uses alpha/beta smoothing. The source constants are
500 accepted updates for convergence, RTT strictly below 10 ms for sample
acceptance, and 100 ms deviation with more than 10 consecutive high-deviation
samples for reset. The comparison uses strict `>` for deviation and count.
A code comment mentions five samples, but the executable constants and condition
are the relevant inspected behavior. Neither threshold enforces the candidate
500 µs registration budget. High-RTT samples do not reset convergence by default.

An important positional detail emerges from reading both implementations.
The XRCE Client calls its callback with `(now, Agent receive, Agent transmit,
Client originate)`. PX4's parameter names do not accurately describe those
positions: its variable named `agent_receive_timestamp` receives Agent transmit
time and is passed into Timesync. In idealized notation, ignoring µs rounding:

```text
T0 = Client transmit on PX4 HRT
T1 = Agent receive on Agent system clock
T2 = Agent transmit on Agent system clock
T3 = Client receive on PX4 HRT
PX4 raw local-minus-remote offset = (T0 + T3)/2 − T2
RTT = T3 − T0
session offset = − filtered local-minus-remote offset
```

The actual callback path uses T2, not both Agent timestamps as the library's
no-callback fallback does. This observation does not authorize changing firmware.
With comparable stable time rates, nonnegative delays, no clock steps and an
instantaneous raw midpoint estimate, asymmetry/processing uncertainty can be
bounded conservatively by RTT/2. A filtered estimate additionally needs its
raw-to-filter difference, quantization and an extrapolation/rate allowance.
At the accepted RTT threshold the conditional half-RTT term alone can approach
5 ms. This is a conditional protocol envelope, not a measured error or a proven
bound for this simulation: PX4 HRT can hold/advance discretely while Agent wall
time advances continuously. A zero simulation-clock RTT cannot prove zero wall
latency. Pauses explicitly violate a simple equal-rate assumption.

The applied session offset is an integer-microsecond cast of the smoothed
estimate, negated and multiplied by 1,000. That cast discards less than one µs
relative to the represented filtered value. It does not certify the filter's
physical accuracy or bound floating-point filter accumulation. Session teardown
resets the filter before connection reinitialization. Normal resynchronization is attempted
on a greater-than-one-source-second schedule, with retries when not converged.
A status record is published inside the filter update, before the callback
assigns the resulting session offset, and is not a serialization sequence marker.

Required missing capabilities are (a) an independently identifiable sensor
acquisition event with both Gazebo header time and PX4 assigned sample time,
(b) an atomic serialization-offset association or validated no-adjustment mode,
and (c) a trusted mapping-generation/session boundary for received data. Existing
coarse diagnostics can reveal changes, but cannot alone bound all three.

## 6. Task E: live-verification decision

Tasks A–D were completed through retained source/build inspection before any
runtime launch. **No live experiment was launched. Zero attempts were used.**
Read-only daemon inspection and copying files from a stopped container are
artifact recovery, not new synchronization measurements.

Another timesync/status window could add empirical RTT or filtered-offset data,
but cannot recover the missing atomic acquisition-event pairing or per-message
correction. Another odometry matching session repeats completed evidence without
validating GPS/IMU acquisition. Reconnection or unsupported Gazebo reset would
not resolve those missing observables. None was performed.

Least-invasive measurement proposals for a separately approved next package:

| Missing observable | Proposed method | Expected evidence and predeclared criterion | Bound on effort |
| --- | --- | --- | --- |
| Acquisition header versus PX4 sample assignment | First establish a unique, timestamp-independent identity between passive Gazebo sensor capture and existing source-side diagnostics/ULog. Include the exported GPS selection path and IMU integration semantics | Matched event pairs with integer times, no ambiguous matches and quantified capture scope. Require total per-stream uncertainty ≤500 µs and worst-case pairwise ≤1,000 µs under an approved horizon/confidence definition. No nearest-time identity | At most two attempts, each ≤180 s including startup/capture, then owned cleanup ≤30 s. If unique identity is unavailable, stop and request approval for a minimal observation hook |
| Recover source time without session-offset ambiguity | Consider the existing reboot-required `UXRCE_DDS_SYNCT=0` runtime parameter in a fresh isolated full session, with `UXRCE_DDS_SYNCC=0` and preserved raw diagnostics | Independently match each permitted topic's source/wire fields and verify zero applied adjustment, correct origin/rate, invalidation and no mixed synchronization modes. This validates transport representation only, not physical acquisition | Separate explicit configuration decision first. At most two attempts of the same duration limit, no firmware rebuild or dependency changes |
| Exact correction if adjusted mode must remain | Export or associate the actual applied serializer offset and mapping generation with each sample, plus complete update provenance | A bounded message-specific inverse across allowed transitions. Nearest status interpolation is insufficient without an update-rate/change bound | Existing interfaces lack this atomic association. No live attempt justified until observation design is reviewed. Any firmware instrumentation needs explicit approval |

These are proposals, not implemented experiments or promised successful bounds.
The existing Compose topology and immutable snapshot should be reused unchanged.
Stop rather than expanding the investigation when an observable is unavailable.

## 7. Task F: candidate contracts and engineering decision

| Contract | Estimator compatibility and measurements | Current evidence / gaps | Complexity and M3.4/M3.5 effect |
| --- | --- | --- | --- |
| A: absolute cross-domain synchronization | Equations can consume it, but require uncertainty-bounded mappings among all required clocks, calibrated rates, asymmetry and transition evidence | No absolute uncertainty bound. Current RTT/filter diagnostics and callback stamping are insufficient | Highest complexity, unnecessary for present event-time GPS mode. Blocks cross-domain readiness if selected. Future M3.5 features needing absolute deadlines must justify their own relations |
| B: run-relative source-clock operation | Compatible if GPS and any compared streams preserve one origin, rate, epochs and validated physical acquisition semantics | Algebra and deterministic epoch controls are sound. Adjusted-wire inversion and GPS/IMU event registration remain unbounded | Lower complexity than A. Host UTC can be removed from the event-time requirement, but M3.4 remains blocked until input topics qualify. Future autonomy still needs separate availability/deadline contracts |
| C: topic-specific restriction using B's time base | Compatible for explicitly allowed topics only, with validated sample semantics and mapping. Full restart defines new epoch. Agent recovery invalidates mapping and requires a fresh isolated session initially | Odometry transformation is characterized for prior matched samples only. No production GPS or cross-stream path currently has a qualifying uncertainty bound | Smallest defensible intended scope. Prevents unsupported relationships from silently entering M3.4. Diagnostic odometry alone cannot substitute for GPS-aided estimator acceptance or authorize M3.5 |

**Recommended decision for review:** select Contract C with Contract B's
run-relative source timeline as the design target. Retain the timing allocations
and the physical acquisition requirement. Do not initialize a production mapping
yet. Keep unsupported GPS/IMU/truth relationships blocked. This is a constrained
path toward the intended estimator, not a readiness PASS for an odometry-only
substitute. There is currently no approved production input whitelist.

Prefer source-time wire transport, if the existing runtime parameter option is
explicitly selected and independently validated. It removes an avoidable
synchronization transform, without changing firmware or removing acquisition
requirements. If adjusted wire timestamps must remain, obtain atomic applied
offset evidence instead. Do not approve a median-offset inverse. Full runtime
restart, fresh epoch, verified publisher/topic identity and generation-specific
validation remain mandatory before input acceptance. During the first supported
contract, pause, reconnect, publisher change or synchronization-mode change
ends acceptance and requires controlled full restart.

No ADR is finalized here because this operating-contract decision remains a
proposal requiring review. No production adapter or new framework is introduced.

## 8. Preliminary readiness, verification and E.4 decisions

| Relationship / gate | Status | Justification |
| --- | --- | --- |
| Same-source-clock relative timing | BLOCKED for production wire inputs | Common-offset cancellation is analytically valid. Full per-message offset variation and physical event registration are not bounded |
| GPS acquisition-time semantics | BLOCKED | Callback-time assignment and selected/blended export path are source-confirmed, but acquisition-header registration is unmeasured |
| Diagnostic IMU timing | BLOCKED for timing-accurate comparisons | Raw and combined timestamp semantics are identified. Integration/event alignment and live errors lack a bound. Independent raw diagnostics remain possible without a timing accuracy claim |
| Odometry timestamp transformation | PASS within the M3.3-D matched capture scope | Addition and per-message age preservation confirmed for 1,140 matches. Absolute accuracy and operational inverse remain BLOCKED |
| Cross-stream alignment | BLOCKED | Shared HRT representation does not prove acquisition alignment |
| Gazebo ground-truth alignment | BLOCKED | No independent event-time match/uncertainty bound. PX4 odometry is an estimate |
| Mapping invalidation | PASS in deterministic validation tests | Explicit invalidation and stale-generation rejection work. Automatic sync-event detection, queued packet provenance and production wiring remain BLOCKED |
| Epoch isolation | PASS in deterministic validation scope | Previous epoch is rejected and controlled sessions carry distinct IDs. No supported in-place reset or physical-registration guarantee is inferred |
| Provisional 0.5 ms per-stream / 1 ms pairwise compliance | BLOCKED | Representation bounds are small, but mandatory acquisition and mapping contributions are unknown |
| Absolute host wall synchronization under proposed restricted contract | NOT_APPLICABLE | No intended event-time operation needs host UTC. Reintroduce the gate if a feature needs cross-domain absolute time |
| Preliminary M3.4 readiness | BLOCKED | No qualifying GPS input path and no complete numerical uncertainty budget. Final approval is reserved for E.4 |

The focused clock/source-to-wire suite passed **33 tests** (22 clock and
11 source-to-wire). Ruff lint and formatting, dependency consistency and
tracked/untracked whitespace checks passed for this documentation package.
No new validation logic was added, so no new deterministic tests were needed.
No estimator sweep, firmware build, Docker runtime campaign or new C++ test run
was performed. Prior E.2 results remain historical evidence rather than tests
re-executed here. Exact verification counts are recorded with the external E.3
inspection evidence.

```sh
/private/tmp/aerotwin-e2-venv/bin/python -m pytest \
  tests/python/test_m3_clock.py tests/python/test_m3_source_wire.py -p no:cacheprovider
/private/tmp/aerotwin-e2-venv/bin/ruff check .
/private/tmp/aerotwin-e2-venv/bin/ruff format --check .
/private/tmp/aerotwin-e2-venv/bin/python -m pip check
git diff --check
```

The existing E.2 environment was reused, with no installation. Supplemental
inspection checked the new untracked Markdown for whitespace and local links.

Before E.4 can make a final readiness decision:

1. Review the proposed Contract C/B scope and the intended GPS-aided use case.
   Approve covered topics, single-GPS selection/blending policy, source epoch,
   physical acquisition/integration definitions and excluded transitions.
2. Approve operational requirements: 500/1,000 µs bounds, V/A envelope, interval,
   observation horizon, worst-case versus probabilistic coverage, accuracy
   allocations and latency/deadline semantics. E.2 alone approves none of these.
3. Select the measurement path: independently validate the existing source-time
   runtime mode, or authorize a precise serializer-offset observable. Any change
   to firmware needs separate explicit approval. No budget relaxation is proposed.
4. Obtain independent acquisition-to-label evidence for every accepted input and
   comparison stream, with identity, completeness limits and a justified total
   uncertainty bound. Exclude any stream that cannot meet the approved scope,
   without claiming that its intended estimator requirement has been satisfied.
5. Demonstrate initialization, fail-closed discontinuity detection, generation
   provenance, stale-buffer handling and controlled restart isolation for that
   exact configuration. Require revalidation after publisher or mode changes.
6. E.4 must assess residual blockers and deployment scope separately, including
   still-unverified Ubuntu AMD64 reference behavior. Do not infer operational
   readiness from ARM64 Docker history or deterministic state-machine tests.

E.3 stopped at this decision proposal. E.4 was subsequently authorized as
review/publication, without authorizing M3.4 implementation or new live work.

## Pinned source references

The inspected copies and generated serializer hashes are retained externally.
The implementation links below pin revisions instead of following a moving tag.

- [PX4 GZBridge clock and sensor callbacks](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/simulation/gz_bridge/GZBridge.cpp#L331)
- [POSIX HRT and lockstep clock implementation](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/posix/src/px4/common/drv_hrt.cpp#L105)
- [Lockstep time assignment](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/posix/src/px4/common/lockstep_scheduler/src/lockstep_scheduler.cpp#L50)
- [PX4 XRCE callback, lifecycle and status](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/uxrce_dds_client.cpp#L58)
- [PX4 outgoing dispatch offset](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em#L124)
- [Synchronization runtime parameters](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/module.yaml#L88)
- [Timesync update and published diagnostics](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/timesync/Timesync.cpp#L47)
- [Timesync thresholds and convergence](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/lib/timesync/Timesync.hpp#L82)
- [Pinned XRCE Client timestamp reply positions](https://github.com/PX4/Micro-XRCE-DDS-Client/blob/711aef423edd1820347b866d1e4164832df35d04/src/c/core/session/session.c#L1171)
- [Pinned XRCE Client HRT time source](https://github.com/PX4/Micro-XRCE-DDS-Client/blob/711aef423edd1820347b866d1e4164832df35d04/src/c/util/time.c#L37)
- [Agent reply timestamps](https://github.com/eProsima/Micro-XRCE-DDS-Agent/blob/73622810d984349b80bbac0ef55fc0b694d62222/src/cpp/processor/Processor.cpp#L723)
- [Agent clock helper](https://github.com/eProsima/Micro-XRCE-DDS-Agent/blob/73622810d984349b80bbac0ef55fc0b694d62222/include/uxr/agent/utils/Time.hpp#L27)
- [GPS selection/export](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp#L129)
- [GPS blended timestamps](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_gps_position/gps_blending.cpp#L376)
- [Combined IMU acquisition definition](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/SensorCombined.msg)
- [Combined IMU sample selection and relative timestamps](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/voted_sensors_update.cpp#L229)
- [EKF2 odometry sample/publication assignment](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L1680)
- [Current AeroTwin observation and lifecycle implementation](../../experiments/python/src/aerotwin_experiments/m3_container.py)
- [AeroTwin epoch/mapping tests](../../tests/python/test_m3_clock.py)
- [M3.3-C evidence and policies](../guides/m3-clock-epoch.md), [M3.3-D matches and limitations](../guides/m3-source-wire.md)
