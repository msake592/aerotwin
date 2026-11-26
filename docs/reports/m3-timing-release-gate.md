# M3.3-E.4 review, architecture decision and release gate

M3.3-E engineering review is complete. **M3.4 production readiness remains
BLOCKED.** Passing tests and timing-sensitivity results do not establish
production timestamp-mapping readiness. The architecture direction and bounded
next package are recorded in [ADR 0011](../adr/0011-source-time-qualification-and-release-gate.md).
Publication is through one PR from `feature/m3-estimator-timing-budget` to main,
with hosted checks reviewed on that PR. No automatic merge is authorized.

## Baseline and scope

PR #28 was freshly verified MERGED at
`9699dee7b3070762d8a3010413a2519466ea12cb`, with all four hosted checks successful.
A remote main lookup still returned that exact revision during E.4 recovery.
The existing feature branch and all E.1–E.3 work were retained. Generated
configurations, CSV/JSON results, plots, logs, source copies, images and build
outputs remain outside the committed package.

E.4 performed read-only model/source inspection, implementation review,
documentation consolidation and regression. No live robotics services were
launched, PX4 rebuilt, firmware modified, robotics dependency installed or
production adapter implemented. The 5,880-run campaign was not repeated.

## Consolidated findings

[E.1](../architecture/estimator-timing-requirements.md) identifies the existing
CV transition and integrated white-acceleration covariance model, one-second
GPS prediction/correction and diagnostic-only 10 ms IMU sampling. The prototype
assumptions V≤5 m/s, A≤2 m/s², per-stream registration ≤0.5 ms and pairwise skew
≤1 ms are approved for continued experimentation only. They are not permanent
system limits or operational guarantees. The proposed local 10 mm / 10 mm/s
accuracy allocations and statistical coverage still require operational review.

[E.2](m3-estimator-timing-sensitivity.md) executed 5,880 runs over 20 seeds,
three motions and separate offset, jitter, rate, acquisition and delivery modes.
The complete repeat matched all CSV fields and aggregates exactly in the same
toolchain. Common-origin offset and ordered delivery latency preserved fixture
results. Accelerating baseline NEES (71.534977, six degrees of freedom) exposes
substantial CV model mismatch, independently of timing errors. Statistical
confidence and worst-case bounds were not established.

Across applicable offset/acquisition/jitter cases, maximum absolute paired
baseline-relative changes are preserved:

| Timing amplitude | Position RMSE change | Velocity RMSE change | Change in maximum position error |
| --- | ---: | ---: | ---: |
| 0.5 ms | 0.877 mm | 0.574 mm/s | 2.281 mm |
| 1 ms | 1.755 mm | 1.147 mm/s | 4.563 mm |

These quantities are differences of per-realization metrics. They are not
maximum pointwise trajectory differences or operational accuracy guarantees.
The original report retains all other sweep values, rejected-event limitations,
covariance observations and the out-of-envelope 100 ms stress result.

[E.3](../architecture/runtime-clock-uncertainty.md) establishes zero added rounding
for checked integer conversion and <1 µs positive ns→µs truncation loss. It
leaves GPS/IMU physical acquisition and message-specific XRCE offset association
unbounded. The 1,140 earlier matched odometry observations characterize the
serializer's offset, not absolute synchronization accuracy. Host UTC can be
omitted for a justified same-source event-time contract, while physical sample
association, rate, epochs and cross-stream alignment remain mandatory.

## Review findings and corrections

| Area | Review conclusion / correction |
| --- | --- |
| Mathematical model | F(h), integrated Q(h), covariance perturbation formulas and local timing bounds agree with production sources and stated assumptions. No core file or estimator algorithm changed |
| Timestamp injection | t=0 is an explicitly unperturbed anchor. Systematic shifts affect subsequent events. Jitter/drift score the original physical target, while stream skew scores the assigned physical event time. These distinct meanings remain documented |
| Paired comparisons | Every configuration retains the same underlying trajectory and GPS noise for its seed. Mean metrics are per-realization means, not pooled RMSE. Reordered delivery omits rejected events and cannot claim improvement from a lower average |
| Truth isolation | Truth generates observations and scores results. An evaluation-only truth-shift probe leaves NIS/covariance/estimated velocity unchanged. Truth is never fed directly into predictor/update APIs |
| NIS/NEES | NIS uses the actual prior and measurement at matching labels. Evaluation-only posterior timestamp projection permits same-physical-target NEES for clock-rate/jitter error. It never feeds back into the estimator |
| Integer arithmetic | Sweep ranges make ns operations safe. Review found unsigned extraction accepted a negative seed. The input parser now validates a signed intermediate against the unsigned range before conversion |
| Metadata | Nonzero extra delay in the reusable reordering control was omitted from its reported delivery delay. Metadata now includes the complete 2 s + extra delay, with a focused test. The historical sweep used extra delay zero and its results are unchanged |
| Determinism | Focused tests now check zero/common-offset equivalence for all three motions, plus invalid motion, drift and seed boundaries. The original complete-sweep equality remains prior evidence. No full rerun was needed for parser/metadata corrections outside its configuration values |
| Hosted coverage | Python-only jobs lack the C++ experiment binary and explicitly skip those integration tests. C++ CI jobs now run the focused Python timing suite after building the executable, on Ubuntu and macOS |
| Architecture/dependencies | The driver links the existing core. ROS/Gazebo remain outside it. Matplotlib is under optional `plots` extras and imported only by plotting. Production runtime dependencies remain empty |
| Documentation | Historical C/D and E.1–E.3 checkpoints are labelled, prototype-only approval is explicit, and roadmap completion is separated from production readiness |
| Artifact exclusion | Only source, tests, CI configuration, build configuration and reviewed Markdown are staged. Numeric summaries are documented; generated raw outputs and credentials are excluded |

### Reviewed files

- Root `CMakeLists.txt`, `pyproject.toml`, `.github/workflows/ci.yml`.
- `examples/timing_sensitivity/CMakeLists.txt` and `main.cpp`.
- `experiments/python/src/aerotwin_experiments/timing_sensitivity.py` and
  `tests/python/test_timing_sensitivity.py`.
- E.1/E.3 architecture documents, E.2/E.4 reports and ADR 0011.
- `docs/guides/m3-source-wire.md`, `docs/guides/m3-clock-epoch.md`,
  `docs/roadmap/ROADMAP.md` and `docs/roadmap/MILESTONES.md`.

## Architecture decision and remaining gate

| Option | Concrete assessment |
| --- | --- |
| A: original PX4 source-clock observations | Practical existing transport qualification through `UXRCE_DDS_SYNCT=0` in a fresh session. No new bridge or firmware build is needed to evaluate source timestamp preservation. Callback-time acquisition semantics still need independent qualification |
| B: exact applied-offset reversal | Requires message-specific offset/provenance instrumentation absent from current status interfaces, a reviewed incremental firmware build and lifecycle validation. It cannot repair discarded acquisition information |
| C: direct Gazebo sensors | Viable alternate architecture, requiring separate approval, adapter transport, frames/geodesy, noise/covariance and lifecycle ownership. Retained NavSat is configured at 30 Hz without explicit noise; IMU at 250 Hz with Gaussian noise. It is not a drop-in replacement for PX4's noisy GPS stream |

Select **A for bounded qualification first**, within a topic-specific run-relative
contract. A source-time mode can eliminate offset reversal, while preserving the
physical acquisition requirement. If physical event association cannot qualify,
C is the relevant alternative; B is reserved for an explicit adjusted-wire
compatibility requirement. No option is currently a production-ready data path.

The exact proposed next package is **M3.3-F.1 — Source-time GPS qualification**.
The ADR specifies interfaces, sample identities, ≤500/1,000 µs requirements,
two independent ≥60-simulation-second sessions, ≤180 s per initial attempt plus
≤30 s cleanup, two-attempt stop policy, unsupported modes and bounded alternatives.
These coverage/configuration choices require approval before execution. A captured
maximum alone does not become a hard bound. Missing event identity, unsupported
noise semantics or incomplete coverage must yield BLOCKED, without budget changes.

The production gate remains blocked by:

1. GPS acquisition-to-label registration and exact selected-topic semantics.
2. Bounded source-time representation or actual applied-offset association for
   every accepted topic, over an approved horizon and coverage definition.
3. Cross-stream/Gazebo event alignment where those comparisons are required.
4. Demonstrated input-session/generation provenance, stale-buffer rejection,
   automatic invalidation and controlled restart handling for the chosen path.
5. Operational confidence, sampling, accuracy and latency requirements, plus
   separate Ubuntu AMD64 reference-host validation.

User decisions required before the next package are approval of the F.1 scope,
no-adjustment configuration, receiver/selection policy and coverage semantics.
Any source observation hook, firmware instrumentation or direct Gazebo bridge
needs its own explicit approval. No such implementation began in E.4.

## Final local regression campaign

| Check | Observed result |
| --- | --- |
| Complete pytest, Debug experiment available | PASS, 119 tests, no skips |
| Focused timing suite within pytest | PASS, 19 cases, covering new parser/metadata boundaries |
| C++ Debug configure and complete build | PASS |
| C++ Debug CTest | PASS, 230/230 |
| C++ Release configure and complete build | PASS |
| C++ Release CTest | PASS, 230/230 |
| Ruff lint / format | PASS |
| Dependency consistency in existing temporary environment | PASS |
| Docker Compose expansion | PASS, three existing services, no privileged mode, no services launched |
| Local documentation links and Git whitespace | PASS |
| Full sensitivity sweep and new live robotics | NOT RUN in E.4, deliberately excluded |

Local execution used Apple Silicon macOS, AppleClang 21, Python 3.14.5 and the
existing E.2 virtual environment. Hosted CI supplies separate Ubuntu/macOS
compile and focused-test coverage. It does not validate the Docker robotics
stack or absolute physical clock uncertainty. Python-only jobs may report
explicit binary-dependent skips, while the C++ jobs execute the focused tests.
The workflow does not install the optional plotting dependency.

Reproduction of the release checks uses the existing presets and environment:

```sh
cmake --preset dev-debug
cmake --build --preset dev-debug --parallel 2
ctest --preset dev-debug
cmake --preset dev-release
cmake --build --preset dev-release --parallel 2
ctest --preset dev-release
/private/tmp/aerotwin-e2-venv/bin/python -m pytest -p no:cacheprovider
/private/tmp/aerotwin-e2-venv/bin/ruff check .
/private/tmp/aerotwin-e2-venv/bin/ruff format --check .
/private/tmp/aerotwin-e2-venv/bin/python -m pip check
git diff --check
```

For Compose validation, supply the existing immutable image ID and unique run,
source and external evidence paths, then use `docker compose config --format
json` only. No image pull or runtime launch is part of this regression campaign.
Review staged paths and local Markdown file links before publication.

## Completion decision

E.1 analysis, E.2 experiments, E.3 uncertainty analysis and E.4 review/ADR are
complete as a consolidated engineering package. The reviewed changes are to be
committed and published as one PR. Hosted CI status and commit identity are
reported on that PR and in the delivery report. M3.4 remains **BLOCKED**, M3.5
has not begun, and M3.3-F.1 requires a separate execution instruction.
