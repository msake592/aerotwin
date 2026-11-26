# ADR 0011 — Qualify source-time GPS before production timestamp adapters

Status: Decided engineering direction for prototype qualification. Execution of
the next package and any production architecture change require separate
approval. M3.3-E.4 closes the analysis/review package; M3.4 readiness is BLOCKED.

## Context

The [E.1 analysis](../architecture/estimator-timing-requirements.md),
[E.2 experiments](../reports/m3-estimator-timing-sensitivity.md) and
[E.3 uncertainty inventory](../architecture/runtime-clock-uncertainty.md)
establish an estimator timing allocation and its sensitivity, but no qualifying
production GPS input. The existing GPS-aided CV estimator uses acquisition times,
one common epoch and calibrated elapsed seconds. Host UTC is unnecessary for
this event-time operation. PX4 estimates remain a comparison path, and simulator
truth remains a separate evaluation input.

The user approved continued prototype experimentation at 0.5 ms per-stream
registration, 1 ms worst-case pairwise skew, 5 m/s speed, 2 m/s² acceleration,
1 s GPS prediction/correction and 10 ms diagnostic IMU sampling. These are not
operational guarantees or permanent limits. No timing allocation is relaxed.

In the retained PX4/Gazebo path, raw GPS and IMU callbacks stamp the current PX4
HRT rather than the Gazebo sensor header. The XRCE serializer additionally
applies a changing session offset. Removing that offset cannot recover an
acquisition header already discarded before uORB publication. Shared units and
common origin do not prove physical sample alignment.

## Options and concrete evidence

### A. Original PX4 source-clock observations

First use the existing reboot-required `UXRCE_DDS_SYNCT=0` option in a new isolated
runtime session, with `UXRCE_DDS_SYNCC=0`. Keep the immutable runtime image,
existing Compose topology, ROS 2 Jazzy message interfaces and source capture.
The client initializes the session offset to zero, and its no-sync callback
sets zero. This provides a practical **transport qualification path**, without a
PX4 rebuild or new source bridge. It is not yet a validated estimator input.

The initial permitted topic candidate is `/fmu/out/vehicle_gps_position`
(SensorGps), with selected receiver/device identity, raw sample and publication
times and synchronization mode retained. Its selection/blending path must be
fixed and verified, initially to a single receiver with blending excluded.
SensorCombined remains diagnostic, with gyro sample time, accel-relative time,
invalid sentinel and integration periods preserved. VehicleOdometry is a PX4
estimate with distinct sample/publication times, not a substitute GPS sensor.

An independent source record and timestamp-independent unique payload identity
are required to verify source-to-wire preservation. For physical registration,
also obtain an identifiable Gazebo acquisition event and its corresponding PX4
assigned sample time. Nearest timestamps and repeated stationary payloads cannot
supply identity. Noise is added in PX4 after Gazebo NavSat reception, complicating
cross-boundary GPS payload matching. A runtime/uORB reader or ULog capture can
preserve original PX4 labels, but cannot recreate a discarded Gazebo header.

A minimal dedicated source-side bridge would add uORB-to-observation transport,
loss/sequence metadata and run/generation identity, with a schema and lifecycle
owner. It remains PX4-specific middleware outside `aerotwin_core`. It can avoid
XRCE inversion, but still needs independent acquisition semantics and uncertainty
validation. Do not build it merely to duplicate existing no-adjustment transport.
If preserving the original Gazebo header needs a source hook, request explicit
firmware/source instrumentation approval first.

Required dependencies are the existing runtime, message package, source logger
and host validation tooling. Numerical conversion is exact in range, but source
physical registration remains unbounded. The first qualification can stop after
two bounded attempts rather than grow into a bridge project.

### B. Exact XRCE reversal

Keep adjusted DDS timestamps and export the exact signed offset **actually used
by each dispatch**, associated with an immutable message identity, publisher,
epoch and mapping generation. Reconstruct `source = wire − applied_offset` with
checked integer arithmetic. The accuracy of that offset as an estimate of UTC
is irrelevant to undoing its own exact arithmetic operation.

Current `timesync_status` gives raw/filtered estimates, RTT and remote time.
DDS export is rate-limited to 10 Hz and has no serializer sequence association.
The client status convergence flag and a median offset cannot supply the
required message-specific metadata. A source-side observation hook or sideband
sample envelope is needed, including atomic correlation and complete offset
update provenance. Reconnect invalidates generation and drains old queues.

The source checkout, compiled artifacts and build caches are preserved. Changing
only XRCE dispatch/observation code could permit an incremental module compile
and relink. This is a feasibility inference, not a timed build result: generated
headers or changed dependencies may cause broader recompilation. No firmware
was modified or rebuilt for E.4. This option requires explicit instrumentation
approval and a fresh binary identity, message correlation tests and live
qualification. It still cannot fix GPS acquisition labels assigned at callback
time. Its incremental cost is therefore unjustified unless adjusted-wire
operation is an explicit requirement that rules out A's existing parameter mode.

### C. Independent Gazebo sensor path

Conceptual path: Gazebo sensor observations → external AeroTwin adapters →
AeroTwin estimator. PX4/ROS 2 remain a parallel reference-estimation/comparison
path. No bridge or adapter is implemented by this decision.

Read-only recovery inspected the retained model repository at
`b6127f4ec20de867e215fb5f78ae88b80f371909`, including x500, x500_base and the
default world. The installed sensor API header reports gz-sensors **8.2.2**,
separate from gz-sim **8.15.0**. These are configured/source observations, not
new measured delivery rates:

| Source | Retained configuration | Consequence |
| --- | --- | --- |
| x500_base NavSat | 30 Hz, no NavSat noise element | Do not treat direct raw NavSat as the same noisy GPS input currently emitted by PX4 |
| x500_base IMU | 250 Hz, zero-mean Gaussian noise, angular-rate sigma 0.0008726646 rad/s each axis; acceleration sigma (0.00637, 0.00637, 0.00686) m/s² | Preserve sensor/body frames, sample time and noise provenance. This differs from the M2 100 Hz diagnostic fixture |
| Default physics | 0.004 s maximum step, requested real-time factor 1 | A 4 ms physics step is neither a measured 4 ms timestamp error nor proof of sub-0.5 ms registration |
| PX4 GPS bridge | Adds its own position/velocity noise after NavSat reception | Direct integration needs a separately approved noise/covariance model and seed policy. Perfect coordinates cannot silently replace noisy GPS |

The version-matched sensor source assigns NavSat and IMU message header stamps
from the update-time argument. NavSat is geodetic position and ENU velocity.
IMU acceleration includes the gravity-related specific-force calculation in
sensor coordinates, with configured noise. Treat frames, gravity sign,
orientation reporting and sensor extrinsics as explicit validation obligations,
not interchangeable with truth or world acceleration. Gazebo header time must
still be validated against the simulator event and data-generation phase.
Transport reception time cannot replace it.

A direct path needs Gazebo Transport/protobuf middleware outside the core,
geodetic-to-local ENU conversion, owned GPS/IMU observation contracts, covariance
and reproducible noise, explicit 30/250 Hz versus 1/100 Hz sampling policy, run
and vehicle identity, stale-data rejection and independent truth capture.
No covariance should be inferred solely from a message type. No timestamp
snapping, unapproved decimation, truth substitution or inertial fusion is implied.

This is viable as a different integration architecture, with more scope than A.
For future multi-agent experiments it adds per-model sensor namespace, identity,
seed and epoch management. A common world clock alone cannot authorize cross-agent
association or network/availability assumptions.

### Comparison and cost

These are rough engineering effort ranges for planning, not measured runtime,
quoted commitments or paid infrastructure requests. They assume the retained
stack works and one engineer already knows the validation infrastructure.

| Option | Expected incremental scope | Planning cost | Principal unresolved gate |
| --- | --- | --- | --- |
| A, existing source-time mode | Validation-only observer/configuration and topic/acquisition qualification | 1–3 engineering days, stop on missing observables | Physical acquisition registration and complete identity; a bridge/source hook is outside this estimate |
| B, applied-offset sideband | Source instrumentation, incremental build/review, correlation and lifecycle qualification | 3–6 engineering days, build duration unmeasured | Atomic metadata plus the same physical acquisition gap |
| C, direct sensor integration | New adapter transport, geodesy/frames, noise/covariance and lifecycle validation | 5–10 engineering days for an initial single-vehicle prototype | Approved noisy GPS model and verified update-time semantics, not automatic budget compliance |

## Decision and bounded next package

Pursue **Option A qualification first**, within the E.3 topic-specific restricted
contract on a run-relative source timeline. It provides a practical existing
transport mechanism and the smallest reversible next step. This selects a
qualification sequence, not production architecture readiness. No production
input whitelist is approved yet. A source bridge is not pre-authorized.

The exact next proposed package is **M3.3-F.1 — Source-time GPS qualification**.
Do not start it automatically. Its acceptance plan is:

1. **Offline gate:** add validation-only capture/schema tests for the existing
   no-adjustment mode, source/wire identity and integer units. Preserve both
   timestamps, device/publisher identity, selection policy, run ID, generation
   and raw capture provenance. Add stale-generation and queued-old-packet tests.
   Keep production estimator mathematics and core dependencies unchanged.
2. **Mode gate:** after explicit runtime-configuration approval, use a new full
   session with recorded parameter values and original binary/image hashes.
   Verify source-to-wire preservation on the actual selected GPS topic, with
   no adjusted-mode packets admitted. Odometry-only evidence cannot pass GPS.
3. **Acquisition gate:** establish independent, unique sensor-event identity and
   quantify actual header-to-assigned-time registration. No nearest-time join,
   stale filtered-offset inverse or assumption that callback time is acquisition
   time is accepted. If the available interfaces cannot identify the events,
   stop with a minimal observation design and request its specific approval.
4. **Coverage gate:** proposed initial coverage is two independent full sessions,
   each with at least **60 simulation seconds** of accepted data, including
   startup logs and a separately identified steady interval. Declare the world,
   seed, CPU/RAM limits and trajectory before capture. Include stationary and
   bounded nonstationary acquisition behavior if a reproducible fixture is
   available without broadening control scope. Otherwise report missing motion
   coverage, not a full qualification PASS. Retain native sensor cadence.
5. **Uncertainty gate:** every accepted stream needs a defensible total
   registration bound ≤**500 µs**, and paired streams need a worst-case bound
   ≤**1,000 µs**, including quantization, event definition, rate/horizon and any
   mapping terms. Combine hard bounds by addition unless statistical assumptions
   are explicitly approved. Record per-run maxima and completeness limitations.
   Two 60 s captures do not prove future worst-case or probabilistic tail bounds.
   A measured maximum alone cannot pass an operational bound.
6. **Bound on execution:** at most two initial attempts for this strategy, each
   ≤180 s wall time including startup/capture, plus ≤30 s owned cleanup. If
   60 simulation seconds cannot be reached within the limit, report inadequate
   coverage. No unbounded retries, package installation, rebuild or transport
   re-verification campaign.

The proposed 60 s horizon, motion coverage and statistical interpretation need
approval with F.1. They do not change the user-approved prototype allocations.
No production latency deadline is approved. Host monotonic reception time is
retained for availability diagnostics, independently of event time.

Use a single agreed source origin across allowed topics, integer simulation
nanoseconds in the core and actual acquisition times. Do not silently convert
30 Hz sensor data into exact 1 s GPS or 10 ms IMU events. An estimator sampling
policy must be specified and reviewed separately before production adaptation.

Each full runtime restart gets a new epoch. Each publisher/mode/session change
invalidates the mapping generation and stops acceptance. Flush or quarantine old
queues; require session/run provenance before assigning a new generation.
Reject old epoch/generation, ambiguous identity, missing metadata, invalid
sentinels, overflow, duplicate or backward samples. Reconnection, pause/resume,
in-place reset, multi-GPS blending, mixed time modes and multi-agent operation
remain unsupported in the initial contract. Agent recovery requires a controlled
full restart and fresh qualification, not seamless continuation.

### Bounded alternatives if A cannot qualify

No alternative starts automatically. A failure caused by lost physical
acquisition semantics is not resolved by B's exact arithmetic inverse.

| Trigger / option | Next bounded feasibility package, after approval | Evidence required / stop condition |
| --- | --- | --- |
| A cannot provide a source-only stream because adjusted timestamps are an explicit integration requirement: B | M3.3-F.B: design an applied-offset envelope for exactly SensorGps and one dispatch path, plus deterministic identity/generation tests. Review cached build impact without rebuilding until instrumentation is approved | One message-specific applied offset and provenance. Missing atomic association or unresolved acquisition identity stops the package. No median inverse fallback |
| A's callback acquisition association cannot meet the budget or recover independent event identity: C | M3.3-F.C: approve a noisy NavSat model and covariance/seed contract, then a passive validation-only capture of NavSat/IMU header, payload, frame and publisher identity from the existing simulator | At most two ≤180 s attempts plus ≤30 s cleanup each, ≥60 simulation seconds per successful attempt. Verify sensor-event stamping and separate truth/noise outputs. No estimator bridge implementation. Missing sensor semantics/noise approval stops the package |

C is the preferred architecture alternative for acquisition information lost in
PX4. B is reserved for a genuine adjusted-wire compatibility constraint. If A
fails only because capture coverage is insufficient, report that exact failure
and seek a bounded revised plan rather than change the allocation.

## Consequences for roadmap and release gate

M3.3-E.1/E.2/E.3/E.4 can close as an engineering package while M3.4 remains
BLOCKED. Final E.4 readiness decision: **do not enable production timestamp
adapters**. Required evidence still includes topic-specific acquisition
semantics, bounded uncertainty over approved coverage, mapping invalidation,
epoch isolation and demonstrated stale-packet rejection for the selected path.

The current repository defines M3 as an umbrella; it does not yet define detailed
M3.5–M3.8 implementation scopes. The following are impact constraints, not newly
implemented or approved roadmap steps:

| Step / future boundary | A or B | Additional changes if C is selected |
| --- | --- | --- |
| M3.4 production timestamp adapters | Gate on actual SensorGps acquisition and lifecycle qualification | Add a separately approved Gazebo sensor adapter/time mapping and geodesy/frame tests |
| M3.5 integration scope, still to define | Keep sensor observations distinct from PX4 reference estimates | Maintain independent AeroTwin sensor and PX4/ROS reference paths; no autonomy or full fusion is introduced |
| M3.6 validation scope, still to define | Preserve replay, timing uncertainty and rejection provenance | Version the new noise model and per-source event semantics |
| M3.7 scenario scope, still to define | Preserve complete run/generation and reset contracts | Define native-rate selection, sensor extrinsics and separate truth capture |
| M3.8 expansion scope, still to define | No implied readiness or automatic expansion | Review multi-vehicle namespaces, clock relationships and seeded sensor instances before implementation |

The standalone C++ core gains no ROS/Gazebo dependency. Python plotting remains
an optional experiment extra. No new bridge, firmware change, production adapter
or future milestone is implemented by this ADR.

## References

- [E.4 review and release gate](../reports/m3-timing-release-gate.md)
- [Pinned PX4 synchronization parameters and timestamp callbacks](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/uxrce_dds_client.cpp#L58)
- [Pinned parameter definitions](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/module.yaml#L88)
- [Pinned GPS callback and noise injection](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/simulation/gz_bridge/GZBridge.cpp#L713)
- [Pinned x500_base sensor configuration](https://github.com/PX4/PX4-gazebo-models/blob/b6127f4ec20de867e215fb5f78ae88b80f371909/models/x500_base/model.sdf#L258)
- [Pinned default physics world](https://github.com/PX4/PX4-gazebo-models/blob/b6127f4ec20de867e215fb5f78ae88b80f371909/worlds/default.sdf#L5)
- [Version-matched gz-sensors NavSat header stamping/noise](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/NavSatSensor.cc#L154)
- [Version-matched gz-sensors IMU sampling](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8_8.2.2/src/ImuSensor.cc)
