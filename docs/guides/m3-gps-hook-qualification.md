# M3.3-F.4 — Proposed GPS acquisition qualification execution gate

Status: **F.4 review complete; execution stopped at the mandatory defect gate**.
GPS qualification and M3.4 remain **BLOCKED**. Zero qualification and overhead
sessions consumed. The [final report](../reports/m3-gps-acquisition-final-gate.md)
records R1: callback lifetime is not fenced before observer state is freed.
The protocol below retains its frozen acceptance rules. The F.4 request supplied
live/overhead/publication approvals, but no firmware repair/build approval;
its material-defect stop rule prevented execution. Historical pending-approval
wording below describes the frozen protocol, not permission to bypass R1.
See the [implementation report](../reports/m3-gps-hook-implementation.md),
[record/build interface](../../integrations/px4/README.md) and authoritative
[F.2 protocol](../architecture/gps-acquisition-observation-hook.md#6-future-qualification-protocol-and-uncertainty).

## Prerequisites and frozen configuration

Obtain explicit F.4 execution approval. Before counting any acceptance data,
verify reference and instrumented image/binary/patch identities, installed Gazebo
library provenance, actual world/model/trajectory, publisher inventory, singleton
raw GPS receiver 0 with fixed nonzero device_id, selected vehicle_gps_position
instance 0 and `px4_msgs/msg/SensorGps`. Record actual fresh-session readbacks:
`UXRCE_DDS_SYNCT=0`, `UXRCE_DDS_SYNCC=0`, `SENS_GPS_MASK=0`, `SENS_GPS_PRIME=0`.
These remain source-confirmed candidate settings, not live-verified values.
Set configured NavSat rate 30 Hz and retain native measured cadence. No blending,
fallback or secondary receiver is allowed. Record SIM_GPS_USED/fix validity,
noise/RNG seed policy, geodesy, frames/extrinsics and no stale-ECM warnings.
Use the retained separate-process Gazebo/PX4 topology; no in-process sensor
publisher. Simulation stamp denotes the NavSat PostUpdate state event, not wall
construction time. Confirm that semantics before interpreting the hook values.

Freeze an external manifest with run/epoch UUIDs, generation, topic/frame,
configuration, installed-library, model/world, upstream, patch, binary and image
digests. Generate the strict context header with `m3_gps_hook context`. Set
`AEROTWIN_GPS_OBSERVER=1` and the context/FIFO paths only in the isolated runtime;
`AEROTWIN_GPS_TIMING=1` is required for an approved timing-probe assessment.
Start the owned FIFO sink before PX4 and retain all headers/footers and failures.
Do not use the original reference container for any instrumented capture.

The existing Compose file does **not** forward these new observer variables or
start its FIFO sink. F.4 must supply and review an explicit run-specific override
and capture manifest, validate expanded Compose, and verify startup ownership.
Its image must be the instrumented digest, not a mutable tag. Source ULog and
ROS wire capture must preserve full precision, raw and selected dataset instance
0, DDS publisher GID, native timestamps and ingress epoch/run provenance. The
F.3 assessor ingests those records; it does not discover publishers, read actual
parameters or provide an automatic ROS/ULog acquisition runner. Do not call
trusted-JSON fixtures real captures. An unreviewed acquisition/export layer or
unverifiable ingress provenance is a stop condition, not a reason to infer identity.

## Attempt and coverage limits

Maximum **two** qualification attempts total, independent full-runtime epochs.
Each attempt includes startup/capture within ≤180 wall seconds, followed by
≤30 seconds owned cleanup. Require ≥60 simulation seconds of steady data,
≥1,800 unique matches per session and no unmatched, ambiguous, invalid, lost,
duplicate or non-forward observations. Explicitly freeze readiness time and a
steady interval starting at least five simulation seconds later. The assessor
requires `readiness_ns`, `steady_start_ns`, `steady_end_ns` for coverage; it
checks all captured events for identity/loss before scoring that interval.
Count/spans/gaps are separate: 1,800 nominal samples can span less than 60 s.
Keep the maximum consecutive gap ≤66,666,668 ns. Require actual acquisition,
assigned HRT, raw, selected and ROS sample spans ≥60 s. Do not snap the
millisecond-throttled Gazebo cadence to an ideal 30 Hz grid.

Include a predeclared reproducible stationary and bounded nonstationary fixture
(≤5 m/s, ≤2 m/s²). Missing approved motion coverage prevents full qualification.
No new control/autonomy campaign, IMU expansion or timing sensitivity rerun.
If a complete session cannot fit the wall limit, report inadequate coverage;
no third attempt or silent shortened horizon.

## Event identity, completeness and epoch rules

Join incoming sensor identity + original sequence within run/epoch, then join
the actual post-noise canonical payload to raw sensor_gps, selected
vehicle_gps_position and ROS. Require full byte equality and uniqueness over
the whole capture on every side. Hashes are indexes; timestamps never resolve
ambiguity. Check exact equality of publication/sample fields only after joining.
Both wire fields must match the original source labels with zero difference.
Retain initialization-only callbacks separately; they publish no GPS measurement.

Require hook callback/enqueued/drained counts to agree, contiguous callback,
record and sensor ordinals, valid header/footer/CRC and zero error/loss counters.
Use passive existing NavSat sequence anchors before/after the steady interval
to detect missing endpoints. No new Gazebo source hook is authorized. ULog
zero-dropout reports alone do not prove one-to-one publication capture.

Full restart gives new run/epoch/session and drained/quarantined queues. Never
relabel unknown queued packets as the new epoch. Reconnect, reset, publisher,
receiver/device or synchronization-mode change invalidates acceptance; initial
pause/resume and in-place reset remain unsupported. A UUID is not proof of
source continuity. Strict JSON validation cannot authenticate incorrectly
attached ingress labels; independently verify the capture boundary.

## Overhead and uncertainty gate

F.3 executed only synthetic offline tests. It provides no measured live overhead
or physical timing bound. Optional probes record Linux monotonic construction
brackets and conservative queue-operation/observed-callback maxima. The latter
starts immediately after the existing HRT read; it does not measure time before
callback entry. A saturated diagnostic maximum is unusable as a finite bound.

The F.2 proposal needs one enabled/disabled comparison pair, ≤20 simulation
seconds per leg, ≤90 wall seconds plus ≤30 cleanup per leg. These are **additional
runtime executions**, requiring separate explicit approval beyond the two
qualification attempts. Without that approval, do not run them or silently
reclassify qualification sessions. If overhead is assessed within the two
attempts instead, freeze a separately reviewed plan that retains all required
instrumented steady coverage; absence of a credible observer term blocks PASS.

Compare declared identical fixture/seed/resources; retain scheduling variability.
Report queue/callback distributions and maxima, CPU, RSS, cadence, high-water,
loss, timer resolution/probe cost and output bytes. Proposed engineering stop
thresholds remain copy/commit observed max 25 µs, added callback p99 50 µs,
RSS increment 8 MiB, zero capture loss/errors and no new ordering anomalies.
These are review thresholds, not approved hard timing allocations or proven
scheduler bounds. No measured p99 or A/B maximum establishes a universal
observer-bias bound, and no observed delay is subtracted from GPS timestamps.

Use checked integers `e_i = 1000 * assigned_hrt_us - acquisition_ns`. Confirm
common simulation origin/rate and epoch from installed lockstep/world-clock
semantics; do not fit offsets or use host reception for mapping. Realized
clock-delivery phase is included in e_i. Combine maximum absolute observed
error with independently justified event, residual mapping, observer and
representation uncertainty by conservative addition. Unknown terms stay unknown.
µs→ns adds zero rounding in range; positive ns→µs truncation is <1 µs and must
not be double-counted as an independent measured error. Require total ≤500,000
ns. An observed error over that value fails immediately; small observations
with unknown terms cannot pass. Pairwise ≤1,000,000 ns requires separately
qualified streams; GPS-only evidence cannot qualify IMU or truth alignment.

The offline assessor always reports physical registration, GPS qualification
and M3.4 as BLOCKED. A subsequent reviewed evidence report, not a synthetic
arithmetic result, must decide any capture-scoped qualification. Finite maxima
do not prove future worst-case compliance, tail probabilities or operational
production readiness.

## Cleanup and stop conditions

Disarm/quiesce subscriptions, drain the owned writer, preserve raw evidence,
checksums, manifests, readbacks, counters, startup and rejection logs. Stop/remove
only owned services/processes/network; retain both reference and review snapshots.
Verify no owned live process remains. On any missing provenance, malformed or
missing footer, overflow, ambiguous/unmatched event, order regression, mode
change, uncertainty gap, failed motion/coverage or deadline, fail closed and
report the exact gate. F.4 does not automatically authorize timestamp fixes,
new hooks, a direct Gazebo adapter, a PR, merge or production M3.4 adapters.
