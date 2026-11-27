# M3.3-F.4 — Final GPS acquisition qualification gate

Decision: **GPS qualification BLOCKED; M3.4 production readiness BLOCKED**.
F.4's mandatory implementation review found a material shutdown-lifetime defect
before execution. **Zero of two qualification attempts and zero of two overhead
sessions were consumed.** No GPS timing-budget PASS or physical timing failure
is claimed. The F.2–F.4 engineering package is published for review with the
firmware defect explicitly unresolved; publication is not acceptance for live use.

## 1. Baseline, authorization and protocol reconciliation

Continued on `feature/m3-gps-acquisition-hook-design` at
`e62966381d939a730d0456bba04e15f59ed5ce56`. PR #30 was freshly verified MERGED,
with all four hosted checks SUCCESS; remote main still resolved to that commit.
All uncommitted F.2/F.3 package files were inventoried and hashed before review.
No redundant branch was created. The authoritative
[F.4 guide](../guides/m3-gps-hook-qualification.md),
[F.2 design](../architecture/gps-acquisition-observation-hook.md),
[F.3 report](m3-gps-hook-implementation.md),
[F.1 report](m3-source-time-gps-qualification.md),
[ADR 0011](../adr/0011-source-time-qualification-and-release-gate.md),
[clock inventory](../architecture/runtime-clock-uncertainty.md) and
[timing requirements](../architecture/estimator-timing-requirements.md) govern
this decision.

The F.4 request explicitly supplies the live, additional-overhead and PR
publication approvals that earlier guide/F.3 text left pending. This
reconciliation was reported before execution. It does not relax the frozen
coverage or timing criteria. The same request prohibits firmware changes and
rebuilds and requires stopping live execution on a material implementation
defect. That stop condition takes effect here; publication of the valid work
and blocked decision remains authorized.

The original guide was copied and hashed before any status annotation. Its
frozen SHA-256 is recorded in the external `attempt-ledger.json`. No acceptance
criterion was changed after examining measurements; there were no live GPS
measurements. Per-run capture boundaries, fixture and Compose override were
not instantiated because the earlier implementation gate failed. This is not
a fully prepared live runner represented as an executed protocol.

## 2. Implementation review: blocking finding

**R1 / P1: Unsubscribe does not establish callback quiescence.**
The unchanged [firmware patch](../../integrations/px4/patches/0001-gps-acquisition-observer.patch)
adds `_gps_observer.stop()` after GZBridge unsubscribes. `Observer::stop` clears
`_enabled`, stops/joins the **writer**, closes the FIFO and resets `_state`.
An already active ticket in `Observer::finish` still dereferences `_state`;
there is no callback admission/lease fence or in-flight callback drain.

At the installed Transport 13.6.0 boundary:

- `src/NodeShared.cc`, `NodeShared::RecvMsgUpdate`, copies `HandlerInfo` under
  the recursive mutex, releases it, then invokes `TriggerCallbacks`.
- `NodeShared::TriggerCallbacks` invokes the copied shared subscription handler
  without that mutex. Retaining the handler does not retain GZBridge/observer
  lifetime: the registered callback targets the GZBridge object.
- `NodeShared::Unsubscribe` removes subscriptions/queued handlers under locks;
  it neither joins the receive callback nor invalidates a copied handler already
  selected for dispatch. `src/Node.cc`, `Node::Unsubscribe`, delegates directly.

Primary source references are the pinned
[receive/dispatch implementation](https://github.com/gazebosim/gz-transport/blob/35fef7ea10c2e133498e9010bd2e397a0ce0c3bf/src/NodeShared.cc#L435),
[unsubscribe implementation](https://github.com/gazebosim/gz-transport/blob/35fef7ea10c2e133498e9010bd2e397a0ce0c3bf/src/NodeShared.cc#L2016)
and [handler callback invocation](https://github.com/gazebosim/gz-transport/blob/35fef7ea10c2e133498e9010bd2e397a0ce0c3bf/include/gz/transport/SubscriptionHandler.hh#L247). This was checked against the **installed
ARM64 library**, not inferred solely from source: RecvMsgUpdate at `0x34ae0`
releases the mutex at `0x34e68` and calls TriggerCallbacks at `0x34ea0`.
TriggerCallbacks is at `0x2e610`; Unsubscribe is at `0x3d350`. The installed
library/package hashes are recorded below and the disassembly is preserved.

A permitted interleaving demonstrates the missing lifetime guarantee:

```text
receive thread                         shutdown thread
copy subscription handler
unlock Transport mutex
enter callback; begin returns ticket
pause before/during finish             Unsubscribe returns
                                       stop writer; join writer
                                       _state.reset()
resume finish -> dereference freed state
```

A copied handler can also start after unsubscribe returns. Clearing `_enabled`
does not revoke an earlier ticket, and an atomic flag cannot protect the
non-atomic unique_ptr lifetime. Footer counters may be sampled before a callback
finishes; a crash or race can prevent meaningful completeness/drainage evidence.
This is a source/binary-confirmed missing synchronization guarantee, **not** a
claim that a live crash or packet loss was observed. No deliberate crash test
or unapproved firmware compilation was performed.

**Correction to F.3:** its claim that the shared receive/unsubscribe mutex made
the separate-process lifecycle quiescent was incorrect. Remote publishing does
not eliminate this race. The report and reproduction README now explicitly
point to this correction. The original patch and image are preserved unchanged
as review artifacts and are not approved for qualification use.

Other reviewed properties remain supported in their stated scope:

| Property | Review result |
| --- | --- |
| Five-file GZBridge-only scope | PASS; exported patch equals the image's complete module diff |
| GPS noise, values, entry HRT and single publication | PASS by additive guarded source diff; no new RNG/conversion/timestamp assignment |
| Actual noisy output identity | PASS offline; native payload bytes copied from the actual SensorGps struct, not regenerated |
| Queue storage and reservation | PASS by inspection/offline tests: 8,192 fixed slots, bounded MPSC one-attempt admission, explicit loss |
| Normal sink framing and error detection | PASS in existing offline scenarios; not proof of safe concurrent teardown |
| Callback lifetime / complete shutdown drain | **FAIL implementation review**, R1 |
| Disabled defaults and truth isolation | PASS by configuration/source/offline tests; no estimator metadata injection |

All 47 focused Python cases and 12 native scenarios were inspected. Native
producer threads are joined before shutdown; normal observer tests call finish
synchronously before stop. They do **not** exercise an active or selected-but-not-
entered callback overlapping destruction. Passing them cannot discharge R1.
No failing test was removed and no firmware fix was applied.

## 3. Runtime provenance and preservation

| Artifact | Full verified identity |
| --- | --- |
| Instrumented snapshot | `sha256:d66443dc476b2626fec1d699b7a5e5f3396dd4d1786dd69b69e6dff2f17c45aa` |
| Instrumented binary SHA-256 | `e135f23733977f56b07b3c63bd1686897d6db8bada09fe483c6b0fb0f1c3f35a` |
| Unchanged firmware patch SHA-256 | `d6768d4b549d302162923b87bbed8ffd2b93d76f3c437bc67368087de21acb13` |
| PX4 upstream | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| Original runtime snapshot | `sha256:7212db2bb758fb941de919bbc5077907de0f773c94bcd415d703383031bf93ef` |
| Original executable SHA-256 | `babe228be6179e8f7abf8af0912ee6b6fae701e9e18cbacfb4d092670ac74402` |
| Original stopped container | `aerotwin-m322-build`, `9a2952433991e12ce0641aa2fc8a7d748e1b95546342bbc6595f6cbf2e7bc369` |
| px4_msgs | `86d8239e962f6939e05c3737784f60c02fa884db` |
| Installed Transport library SHA-256 | `cc52b779cf78bd891681c12f8f117234cbd9f8b04bab1f9f9026e879fcc545ed` |

F.4 rechecked the full image IDs, labels, actual executable, applied diff digest,
Gazebo library hashes and package-integrity results against F.3. NavSat 8.2.2,
simulation 8.15.0, Transport 13.6.0 and msgs 10.4.0 package identities and hashes
match the implementation report. ROS 2 Jazzy and the pinned px4_msgs SensorGps
class imported successfully with the expected fields; this is interface
compatibility evidence, not live DDS discovery or timestamp preservation.

Docker was available: 12 CPUs / 8,319,172,608 bytes VM RAM, approximately 7.2 GiB
available and 401 GiB overlay free. The owned `aerotwin-f4-review` container used
the immutable instrumented image, read-only root, no network, two CPUs / 2 GiB,
a bounded temporary filesystem and a read-only Python source mount. Its only
work was introspection and the existing precompiled synthetic test executable.
It launched **no PX4 process, simulation world, Agent or ROS observation node**.
It is not an overhead or qualification session.

Original container metadata and original executable hash were compared before/
after review. Both original and instrumented snapshots remain intact, with the
observer disabled by default. The owned review container was stopped within its
owned cleanup allowance; no containers remained running at verification. Live
observer shutdown/cleanup was deliberately not exercised because it is the
blocking defect. No firmware source, image, package or compiled artifact changed.

## 4. Frozen policy and attempt accounting

The guide's limits remain: candidate `UXRCE_DDS_SYNCT=0`, `UXRCE_DDS_SYNCC=0`,
`SENS_GPS_MASK=0`, `SENS_GPS_PRIME=0`; singleton raw receiver 0, fixed nonzero
device ID; selected `/fmu/out/vehicle_gps_position` SensorGps; configured 30 Hz.
These settings remain **source-confirmed candidates**, not actual readbacks.
The unchanged coverage rules require two independent full epochs, ≥60 simulation
seconds and ≥1,800 matches per session, readiness + five simulation seconds
startup exclusion, gap ≤66,666,668 ns and zero ambiguous/unmatched/lost records.
Native cadence is measured rather than snapped; identity uses canonical bytes
before checked integer timestamp comparison. Passive sequence anchors, full
run/epoch provenance and exact no-adjustment timestamp equality remain mandatory.

Qualification limits are two attempts, ≤180 wall seconds each plus ≤30 seconds
owned cleanup. Additional overhead limits are the approved enabled/disabled
pair, ≤20 simulation seconds and ≤90 wall seconds plus ≤30 cleanup per leg.
These cannot replace qualification attempts. Observer probes and CPU/rate
proxies do not supply a hard acquisition-error bound.

| Planned execution | State | Live count / simulation span | Attempt consumed |
| --- | --- | --- | --- |
| Overhead disabled | NOT RUN: review gate failed | Not measured | No; overhead used 0/2 |
| Overhead enabled | NOT RUN: review gate failed | Not measured | No |
| Qualification 1 | NOT RUN: review gate failed | Not measured | No; qualification used 0/2 |
| Qualification 2 | NOT RUN: review gate failed | Not measured | No |

The external ledger contains empty actual-attempt/session lists, limits,
stop reason and frozen guide digest. These are **not four failed live sessions**.
No start/end window was selected, no boundary samples were dropped, and no
retrospective exclusion or silent retry was used to obtain a PASS. No raw live
GPS dataset exists for F.4. Synthetic records are not promoted to live evidence.

## 5. Association, statistics and uncertainty

There are no live event/source/wire associations. Counts, timestamp differences,
minimum, maximum, maximum absolute difference, median, p95/p99 and variation
versus simulation time are all **NOT MEASURED** for both planned qualification
sessions. No runtime receiver identity, complete capture, exact preservation,
noise trajectory or independent live epoch result is asserted.

The intended checked calculation remains
`e_i = checked(1000 * assigned_hrt_us - acquisition_ns)` after unique event and
canonical payload association. A common simulation epoch/origin/rate must be
established from the installed lockstep/event semantics. Do not fit an offset,
use reception clocks, use nearest timestamps or substitute earlier odometry
transformation residuals. Post-label transport latency is not acquisition error;
clock-delivery phase before callback-time assignment can affect e_i.

| Contribution | Usable result in F.4 |
| --- | --- |
| Gazebo acquisition/ECM state semantics | Source/installed metadata available; live event-state scope not qualified |
| Assigned HRT / clock mapping / scheduling | Source-confirmed callback assignment; realized registration difference unmeasured |
| Checked in-range µs→ns conversion | Analytically exact, 0 ns added rounding |
| Positive ns→µs representation | Analytically <1,000 ns; no double counting as a second measured error |
| Observer-induced uncertainty | Unknown; no A/B sessions; lifetime defect invalidates the planned shutdown evidence |
| Capture completeness | BLOCKED by R1; no live captures |
| Combined defensible uncertainty | **UNKNOWN**; cannot compare a finite total to 500,000 ns |

The 500 µs per-stream / 1 ms applicable pairwise budgets remain unchanged.
No required unknown term is replaced by zero or inferred from CPU/rate similarity.
An observed maximum would only describe captured events under validated reference
assumptions, not a universal scheduling bound or future operational compliance.
GPS-only evidence cannot qualify IMU or Gazebo truth alignment.

## 6. Independent release gates

| Gate | Assessment | Evidence type and supporting artifact |
| --- | --- | --- |
| Instrumented runtime provenance | PASS | Read-only image/labels/source/diff/binary/library checks; runtime-preflight.log and instrumented-image.json |
| Original runtime preservation | PASS | Before/after container manifest and original executable SHA-256 |
| Actual no-adjustment configuration | BLOCKED | No live readbacks; candidate settings only |
| Actual GPS receiver identity | BLOCKED | No live inventory/device/selection observations |
| Acquisition event semantics | BLOCKED | Source/binary metadata availability confirmed; no qualified live state-event evidence |
| End-to-end event association | BLOCKED | No live event/source/wire data |
| Source-to-wire timestamp preservation | BLOCKED | Offline checks pass; no live GPS preservation window |
| Capture completeness | BLOCKED | Material lifetime review FAIL; no trusted live footer/counts |
| Required session coverage | BLOCKED | 0/2 sessions executed, not measured counts/spans |
| Observer-induced uncertainty | BLOCKED | No live assessment or defensible bound |
| 500 microsecond registration compliance | BLOCKED | Total uncertainty unknown; not a measured timing FAIL |
| Epoch isolation | BLOCKED for live scope | Existing deterministic Mapping/record tests PASS; no independent live session pair |
| Stale-record rejection | PASS, deterministic validator scope only | Existing wrong epoch/generation/publisher/build, duplicate and backward-order tests; ingress provenance remains unverified |
| Runtime cleanup | PASS, review-container scope only | Owned review container stopped; no live GPS session cleanup claim |

All mandatory GPS gates are not satisfied. **Do not enable any restricted M3.4
GPS adapter.** F.2 design/F.3 build/test evidence remains useful, but F.3's
implementation is not accepted for live qualification by this review.

## 7. Regression, reproduction and evidence

Final local verification passed: **196 Python tests** (47 hook cases),
**230/230 Debug CTest**, **230/230 Release CTest**, and all **12 existing native
observer scenarios**. Native tests used the already compiled executable in the
snapshot; there was **no firmware or native recompilation**. Ruff lint/format,
dependency consistency and Compose expansion passed. Documentation/local links,
patch-content and Git whitespace checks passed before publication. These checks
cannot substitute for the missing callback lifetime test or live evidence.
The estimator sensitivity campaign was not repeated. No dependency was installed.

Reproduce permitted host checks using the existing environment:

```sh
/private/tmp/aerotwin-e2-venv/bin/python -m pytest -p no:cacheprovider
cmake --build --preset dev-debug --parallel 2
ctest --preset dev-debug
cmake --build --preset dev-release --parallel 2
ctest --preset dev-release
/private/tmp/aerotwin-e2-venv/bin/ruff check .
/private/tmp/aerotwin-e2-venv/bin/ruff format --check .
/private/tmp/aerotwin-e2-venv/bin/python -m pip check
git diff --check
```

Native offline rerun used `python3 /tmp/run_observer_tests.py /tmp/observer_test`
in the read-only review container, with `TMPDIR=/run/f4`,
`PYTHONPATH=/aerotwin-parent`, `PYTHONDONTWRITEBYTECODE=1`. No build commands were
used in F.4. The frozen guide and F.3 reproduction commands are retained as
historical specifications; do not execute their firmware build steps under
this F.4 authorization.

Raw inspection/disassembly and verification evidence is outside Git under
`/private/tmp/aerotwin-f4-evidence`; F.3 evidence remains independently preserved.
Key files are attempt-ledger.json, frozen-guide.md, work-before.json,
runtime-preflight.log, ros-interface.log, instrumented-image.json,
original-image.json, original-container-before/after.json,
transport-source-review.txt, transport-dispatch-unsubscribe.txt,
installed-transport-disassembly.log, pytest.log, native-tests.log,
debug/release build/CTest logs, compose.json and the final manifest.json.
The manifest records file digests and the publication identities separately.
No binary, image, build cache or raw runtime evidence is committed. Temporary
host evidence must be archived before cleanup of /private/tmp.

## 8. Architecture recommendation and required decision

Classify the stop as a **correctable observation-lifecycle implementation
defect**, not evidence of a fundamental PX4 timing limitation. No acquisition
measurements exist to decide whether callback-time HRT can meet 500 µs.

Recommend one bounded intervention for explicit approval: repair callback
admission/lifetime and rerun its offline review gate on the same upstream pin,
then decide whether the already bounded qualification plan may resume. The
reviewable repair requirements are:

1. A lifetime-owned callback gate must be captured by registration, close new
   admissions before teardown and reject a previously selected late callback
   **before it accesses GZBridge or observer state**. Counting only callbacks
   that have already entered is insufficient.
2. Each admitted callback holds an owned lease through its entire callback and
   finish path. Teardown waits for accepted leases before footer/drain/free.
   Timeout must fail capture without freeing still-accessible state, detaching
   a thread or blocking the GPS callback.
3. Deterministic tests must force selected-before-unsubscribe/entered-after-close,
   active-finish versus stop, and producer/consumer/shutdown overlap. Disabled
   behavior, GPS values/noise/timestamps and one publication must remain unchanged.
4. A separately approved five-file GZBridge-area patch, incremental build and new
   patch/binary/image identities must pass review. No repair is applied here.

Planning comparison, not measured implementation cost: a focused lifecycle
repair/test/review is approximately **1–3 engineering days**, subject to proving
the teardown ownership contract; a direct noisy Gazebo GPS path remains the
ADR's broader **5–10 day** initial prototype with new adapter/geodesy/noise/
covariance/seed contracts. The latter bypasses callback-time source labels but
does not inherit PX4's noisy measurement model or automatically pass timing.
Do not substitute perfect raw NavSat for noisy GPS. If lifetime repair requires
broader ownership changes, return that scope for approval before implementation.

This recommendation does not authorize a new firmware patch, rebuild, live
session, direct architecture, production adapter or another numbered clock
investigation package. User approval is required for the concrete repair above.
No F.5 is created and no automatic merge occurs. The consolidated PR explicitly
publishes a **BLOCKED qualification outcome** and the unresolved P1 finding.
