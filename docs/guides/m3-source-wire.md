# M3.3-D source-to-wire timestamp verification

This guide preserves the M3.3-D checkpoint and its measured scope. The
subsequent [M3.3-E release gate](../reports/m3-timing-release-gate.md) records
prototype timing allocations and the current architecture decision. Production
timestamp mapping remains BLOCKED.

## Capture and identity

The retained default ULog inspected for this package contained no odometry
dataset. The pinned logger supports a runtime topic configuration at
`etc/logging/logger_topics.txt`. Opt-in `AEROTWIN_SOURCE_CAPTURE=1` copies
the existing startup assets into the owned session workspace and configures
`vehicle_odometry 0`, `timesync_status 0` and `sensor_combined 0`.
Firmware binaries and source remain unchanged.

The ULog parser reads instance 0 of the actual `vehicle_odometry` uORB topic,
with publication and acquisition times and binary float32 payload fields.
ROS captures use the existing observer. Matching excludes both timestamps.
Canonical little-endian float32 position, velocity and Hamilton quaternion
components, pose/velocity frame enums and reset counter define the identity
within a capture. Signed zero is preserved. Nonfinite, missing, malformed,
unsupported-frame and invalid-orientation observations are rejected.

SHA-256 indexes the canonical payload. A match requires uniqueness in both
captured streams and a byte-for-byte canonical payload comparison. Duplicate
fingerprints and mismatched epoch/generation contexts are rejected. No nearest
timestamp or candidate offset contributes to identity.

Uniqueness is scoped to the captured data. ULog sampling can miss publications,
and zero logger dropouts cannot prove that every uORB update was recorded.
Repeated payloads among uncaptured publications remain an identification
limitation. Angular velocity and variance fields are excluded from this
fingerprint. The scoped finite position/velocity/orientation payload provides
the measured correspondence.

## Acceptance defined before independent verification

The pilot needed defensible unique matches before further sessions. The
independent sessions require at least three unique pairs over multiple updates
for transformation characterization. Every ambiguous captured fingerprint is
rejected. Publication and acquisition offsets are calculated independently
and compared with the serializer's common-offset behavior.

No numerical estimator timing-error budget currently exists in the AeroTwin
contracts. Therefore no observed residual threshold authorizes mapping
initialization. All reports retain `mapping_valid=false` and unknown absolute
uncertainty. Residuals describe observed offset variation.

## Results

The pilot produced 560 matches and three unmatched wire observations.
Two independent sessions then produced the following measurements.

| Measurement | Session 1 | Session 2 |
| --- | --- | --- |
| Source ULog rows | 1,709 | 1,686 |
| Unique matched observations | 575 | 565 |
| Unmatched wire observations | 4 | 11 |
| Matched source span | 5,744,000 us | 5,640,000 us |
| Publication offset minimum | 1,795,554,955,984,231,000 ns | 1,795,554,976,125,889,000 ns |
| Publication offset median | 1,795,554,955,984,231,000 ns | 1,795,554,976,125,892,000 ns |
| Publication offset maximum | 1,795,554,955,984,232,000 ns | 1,795,554,976,125,893,000 ns |
| Absolute median residual p95/max | 1,000 / 1,000 ns | 3,000 / 3,000 ns |
| Publication minus acquisition offset | 0 for every pair | 0 for every pair |
| Matched wire minus source span | 0 us | 2 us |
| Ordering violations | 0 | 0 |
| Logger dropouts | 0 | 0 |
| Source sampling intervals | 4–12 ms | 4–12 ms |

Acquisition offset distributions equal the publication distributions in both
sessions. Median and p95 are integer order statistics. Span difference
characterizes observed clock-rate consistency over the captured window.
Neither that difference nor offset residuals supplies an absolute
synchronization guarantee.

Both sessions used distinct runtime epochs and publishers and passed
transport and cleanup. Source and wire rows are joined only within their
session. No new pause, Agent interruption or Gazebo reset was performed.
Existing evidence for those transitions remains applicable as a limitation.

The positive wire-minus-source offset agrees with the pinned generated
VehicleOdometry serializer, which adds the XRCE session offset to both
timestamps. See [timestamp source semantics](m3-clock-epoch.md#source-semantics).
This result applies to VehicleOdometry in the observed runtime mode.

## Reproduction

Activate the editable host Python environment. The existing runtime snapshot
provides pyulog through `/work/venv/bin/python`; host Python needs no new
robotics dependency. The ingestion job runs without networking after the
owned simulator services stop.

```sh
python -m aerotwin_experiments.m3_source_wire_validation \
  --image sha256:7212db2bb758fb941de919bbc5077907de0f773c94bcd415d703383031bf93ef \
  --evidence "$NEW_EXTERNAL_EVIDENCE_DIRECTORY"
```

The CLI saves runtime reports, raw ULogs, source rows, wire JSONL, matches,
epoch/publisher context and measured offset distributions. A `complete`
marker identifies finished capture/analysis. Preserve raw evidence when
recomputing analysis with revised rejection rules.

## Readiness

Source capture, observed payload correspondence and both timestamp
transformations pass within the captured streams. Independent sessions
reproduce the transformation mechanism. Epoch/generation isolation remains
enforced by the existing validation infrastructure.

Absolute uncertainty, sampling completeness and an estimator timing tolerance
remain unresolved. M3.4 is **BLOCKED**. Define the timing-error budget and
quantify source capture and synchronization uncertainty before authorizing
production timestamp mapping. Unsupported in-place reset and unvalidated
reconnection continuity remain blocked.

## Regression verification

The local suite passed 100 pytest cases, including eleven focused payload-identity
tests. Debug and Release each passed 230 CTest cases. Ruff lint/format,
dependency checks, expanded Compose configuration and whitespace checks passed.
The CI tests use synthetic payloads. The numerical offset measurements above
come from the preserved independent live captures.

```sh
python -m pytest -p no:cacheprovider
ruff check .
ruff format --check .
python -m pip check
ctest --preset dev-debug
ctest --preset dev-release
git diff --check
```
