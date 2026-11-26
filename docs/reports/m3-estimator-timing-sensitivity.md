# M3.3-E.2 estimator timing sensitivity experiments

Status: **completed for review**, with provisional requirements. M3.4 remains
**BLOCKED** pending independently validated clock uncertainty and approved
operational requirements. No E.3 work, Docker execution, firmware changes or
production estimator changes were performed. No PR was opened during E.2. The consolidated package is reviewed for
publication in [E.4](m3-timing-release-gate.md). The listed timing/motion values
are now approved for continued prototype experimentation only, with no
operational guarantee.

## Recovery and scope

The existing branch is `feature/m3-estimator-timing-budget`, based on
`9699dee7b3070762d8a3010413a2519466ea12cb`. PR #28 was verified merged at this
revision with successful hosted checks. The uncommitted
[E.1 analysis](../architecture/estimator-timing-requirements.md) is preserved.
Recovery found no earlier E.2 implementation to resume. This package adds an
experiment executable, Python orchestration, focused tests and this report.
Generated configurations, results and plots remain outside version control.

## Method and source references

The C++ [experiment driver](../../examples/timing_sensitivity/main.cpp) calls the
production [predictor](../../core/src/estimation/KalmanPredictor.cpp),
[GPS updater](../../core/src/estimation/GpsKalmanUpdater.cpp),
[GPS simulator](../../core/src/sensors/GpsSensor.cpp) and
[metrics](../../core/src/validation/EstimationMetrics.cpp).
The [Python module](../../experiments/python/src/aerotwin_experiments/timing_sensitivity.py)
creates the sweep, invokes one bounded batch process, computes paired changes,
and writes CSV, JSON and plots. A failed subprocess aborts result generation.

The experiment retains the M2 [acquisition configuration](../../examples/fusion_demo/ScenarioAcquisition.cpp)
and [evaluation model](../../examples/fusion_demo/FusionEvaluation.cpp):

- World positions and velocities use ENU, metres and seconds. All events share
  epoch zero. GPS acquisitions and corrections occur at 0, 1, 2, 3 and 4 seconds,
  with four predictions. The first correction anchors time correctly in every
  trial. Perturbations affect subsequent events.
- Initial truth position is (10, −5, 2) m. Stationary velocity is zero. CV velocity
  is (2, 1, 0.5) m/s. Accelerating truth uses initial velocity (−4, 0, 0) m/s and
  constant acceleration (2, 0, 0) m/s², hence at most 4 m/s over the nominal run.
  Analytic reference evaluation extends to shifted acquisition endpoints, where
  the largest sampled truth speed is 4.2 m/s.
- The filter estimates position and velocity, with constant-velocity transition
  F = [[I,hI],[0,I]]. It has no acceleration state. Continuous white-acceleration
  density D = 0.01 I m²/s³ gives Q blocks h³D/3, h²D/2 and hD.
- Initial estimated state is zero with P = diag(100,100,100,4,4,4).
  GPS covariance is I m². Seeds 42 through 61 give 20 deterministic realizations
  per configuration. Noise is generated once at each nominal acquisition and
  reused between paired scenarios, including shifted acquisitions.
- The M2 10 ms IMU cadence remains diagnostic context. This harness does not
  synthesize or fuse IMU samples. GPS changes are measured against the common
  reference clock, serving as a proxy for registration to another stream.

Ground truth generates simulated sensor observations and scores results. It is
never supplied to the predictor or updater. An evaluation-only `score_only`
probe shifts scoring position by 100 m and verifies that NIS, covariance and
estimated velocity metrics remain unchanged. It is excluded from the sweep.
The accelerating scenario deliberately exposes model mismatch in the CV filter.

### Distinct time meanings

| Mode | Acquisition, assigned label and scoring behavior |
| --- | --- |
| Baseline | Measurement z(t), prediction/update at t, scoring truth at t |
| Common origin | Raw integer origin 10¹⁸ + e ns is shared and subtracted before converting intervals to seconds. Elapsed labels remain t |
| Stream offset | Preserve z(t), assign it to t+e, predict/update at that label and score against truth at t+e. This tests incorrect association with the reference clock |
| Jitter | Preserve z(t), assign t+j(t), propagate using perturbed intervals and score against the original physical target t |
| Drift | Preserve z(t), assign t + tρ, propagate using the wrong clock rate and score against original physical target t |
| Acquisition shift | Acquire z(t+e), assign label t and score against truth at t. Noise is shared with baseline |
| Delivery delay | Deliver at t+L, retain acquisition/label t and process in order. Score the event-time state at t |
| Reordering control | Delay only the t=1 s delivery by 2 s, making it arrive after the t=2 s event. The forward predictor rejects it |

For jitter and drift, an **evaluation-only** copy of the posterior carries the
physical target timestamp for aligned RMSE/NEES calls. Its state vector and P
are unchanged and the copy is never fed back to the filter. This explicitly
compares an erroneous clock interval against its original physical target.
Stream-offset scoring instead retains the assigned physical event time, as
specified by E.1. Differences between these modes include their scoring target.
NIS always uses the actual prior and measurement with matching assigned labels.
NEES uses posterior error and P, with six degrees of freedom. NIS has three.
No production timestamp equality or epoch contract is relaxed.

Events are stably ordered by delivery time. Ordered latency is an offline
availability experiment, without a real-time deadline or present-time output
accuracy claim. No rollback, interpolation or out-of-sequence correction is
implemented. All non-control trial labels remain strictly forward.

## Sweep and artifacts

There are **5,880 runs in 294 configuration groups**, each group containing
20 seeds. Applicable amplitudes are 0, 1 and 3 µs, then 0.1, 0.5, 1, 5, 10, 50
and 100 ms. Stream offsets, acquisition shifts and common-origin changes use
both signs. Latency and jitter amplitudes are nonnegative. Jitter uses integer
samples from `mt19937_64(seed + 10000)` mapped into the declared symmetric bound.
At the 0.5 ms amplitude the observed extrema are −497,966 and +490,251 ns.
At 1 ms they are −943,323 and +990,197 ns. CSV rows record each realization's
actual minimum and maximum, including the anchored zero-time event.
This is a bounded deterministic perturbation ensemble, without a Gaussian or
zero-mean sample claim.

Drift uses signed rates 0, 1, 10, 25, 100, 125, 250, 1,000, 12,500 and 25,000 ppm.
The first values sample small clock-rate errors. The larger rates explicitly
stress the model and span the same endpoint errors as the timing sweep.
Over four seconds their absolute accumulated errors are respectively 0, 4, 40,
100, 400, 500, 1,000, 4,000, 50,000 and 100,000 µs.
Integer arithmetic calculates mapped times before conversion to seconds.

Output files are `configurations.json`, `results.csv`, `summary.json` and four
PNG plots for position RMSE, velocity RMSE, NIS and NEES. CSV includes units,
drift rates, delivery delays, observed perturbations, paired metric deltas,
accepted/rejected counts, covariance traces, minimum eigenvalue, symmetry error
and speed diagnostics. Summary values average per-realization metrics. They
are not pooled RMSE. Plots show seed means for systematic offsets, acquisition
shifts, jitter bounds and delivery delays. Drift has separate units and is
reported below rather than placed on the millisecond sweep axis.

## Observations

### Correct-timestamp baselines

| Motion | Position RMSE (m) | Velocity RMSE (m/s) | Mean NIS (dof 3) | Mean NEES (dof 6) |
| --- | ---: | ---: | ---: | ---: |
| Stationary | 1.497565 | 1.056251 | 2.290286 | 4.714086 |
| Constant velocity | 1.515160 | 1.495668 | 2.581951 | 4.968990 |
| Accelerating | 1.717356 | 3.055696 | 4.888388 | 71.534977 |

These include initialization transients and five posterior events per run.
Accelerating baseline NEES is much larger than its degrees of freedom,
consistent with substantial CV model mismatch. Timestamp perturbations are
assessed relative to that same baseline. Twenty fixed seeds with correlated
within-run events do not establish confidence intervals or a chi-square
consistency acceptance test.

### Largest paired changes

Each entry below is the maximum **absolute change from its paired baseline**
over motions, seeds and both systematic signs at the stated amplitude.
The maximum-position column is a change in each run's maximum position-error
norm. It does not bound the maximum pointwise difference of two trajectories.

| Mode | Amplitude (ms) | Δ position RMSE (mm) | Δ velocity RMSE (mm/s) | Δ maximum position error (mm) |
| --- | ---: | ---: | ---: | ---: |
| Stream offset | 0.5 | 0.877 | 0.574 | 2.281 |
| Stream offset | 1 | 1.755 | 1.147 | 4.563 |
| Stream offset | 5 | 8.790 | 5.740 | 22.833 |
| Stream offset | 100 | 183.181 | 116.569 | 464.421 |
| Acquisition shift | 0.5 | 0.649 | 0.307 | 1.561 |
| Acquisition shift | 1 | 1.299 | 0.615 | 3.123 |
| Acquisition shift | 5 | 6.497 | 3.074 | 15.636 |
| Acquisition shift | 100 | 136.114 | 61.771 | 323.526 |
| Jitter bound | 0.5 | 0.214 | 0.146 | 0.724 |
| Jitter bound | 1 | 0.505 | 0.273 | 1.518 |
| Jitter bound | 5 | 1.396 | 1.675 | 4.372 |
| Jitter bound | 100 | 42.207 | 32.918 | 81.285 |

At 1 ms stream offset, the largest absolute changes in mean NIS and mean NEES
are 0.001553 and 0.081651. At 1 ms acquisition shift they are 0.005473 and
0.018291. Changes can have either sign with the paired noise realization.
Improvement of one sample metric does not establish a beneficial timing bias.

Drift rates ±125 and ±250 ppm reach 0.5 and 1 ms at the last event. Their largest
absolute paired changes in position RMSE are 0.058 and 0.116 mm, velocity RMSE
0.119 and 0.237 mm/s, and maximum position error 0.192 and 0.385 mm.
At ±25,000 ppm (100 ms endpoint error), these become 11.889 mm, 23.990 mm/s
and 39.254 mm. A rate budget must specify its validation horizon: this four-second
result does not validate the illustrative ten-second E.1 allocation.

All common-origin and ordered-latency trials reproduce baseline metrics exactly,
including covariance and NIS/NEES. Stationary acquisition shifts are also
invariant. Stationary label errors can still matter because noisy updates create
nonzero estimated velocity. Systematic stream association errors produce the
largest RMSE sensitivities among the tested cases. Acquisition misalignment
and rate errors remain relevant even with a shared run-relative origin.

All ordinary runs accept five measurements with zero ordering violations or
rejections. Each of the 60 reordering runs accepts four and rejects one for
non-forward time. Their metrics exclude the rejected event and use a different
sample count, so a smaller reported RMSE would not establish improved filtering.
Across all trials P stays symmetric to reported precision and the smallest
posterior eigenvalue is 0.03732781. Covariance positivity does not imply calibrated
uncertainty or physical timing correctness.

The largest estimated speed within the candidate (≤1 ms) cases is below 5 m/s.
The 100 ms stream-offset stress case reaches 5.147343 m/s at seed 55 during
accelerating motion. That stress observation extends beyond the provisional
estimated-speed envelope. Gaussian measurements and the production filter do
not enforce hard speed limits.

## Candidate-budget assessment

The empirical results **support retaining 0.5 ms per stream and 1 ms pairwise
skew as provisional candidates** for this one-second GPS experiment. Across
stream-offset, acquisition-shift and bounded-jitter trials up to 1 ms, the
largest absolute paired changes are 1.755 mm position RMSE, 1.147 mm/s velocity
RMSE and 4.563 mm maximum position error. These are small against the proposed
10 mm / 10 mm/s timing allocations. Drift at the corresponding endpoint error
also remains small in these four-second trials.

Those allocations are local physical association bounds in E.1, not approved
end-to-end RMSE or posterior error limits. Here they are comparison proxies.
E.1 independently gives the local bound 5.001 mm displacement and 2 mm/s velocity
misalignment at 1 ms under V≤5 m/s and A≤2 m/s². The largest cubic Q coefficient
change at a 1 ms interval perturbation around one second is about 0.3003%, below
the provisional 1% allocation. These analytical and empirical statements must
retain their different meanings.

There is no evidence in this fixture that the candidate is insufficient.
There is also insufficient evidence to declare it unnecessarily strict or to
approve a looser operational budget. At 5 ms, systematic offsets and acquisition
shifts already change maximum position error by more than the 10 mm comparison
allocation. No threshold was adjusted after observing the results.

A common constant origin cancels when every participant shares it. Independent
stream offsets, rate errors, jitter and physical acquisition misalignment
survive origin subtraction. Accurate acquisition labels preserve ordered offline
results despite latency. Absolute wall-clock synchronization is unnecessary for
these equations, while correct relative origins, seconds calibration, epochs
and physical registration remain necessary. A GPS-only experiment cannot
validate actual cross-sensor alignment or latency deadlines.

The prior 1,140 payload matches and 1/3 µs maximum median-relative source-to-wire
residuals remain evidence of serializer-offset variation. They are **not absolute
synchronization error bounds** and are not used to certify this candidate budget.

## Reproduction and verification

Use existing C++ build prerequisites. The optional `plots` Python extra adds
Matplotlib for standard host-only scientific plots. It is unnecessary for
non-plot experiment execution. This run used a temporary virtual environment,
with no global or robotics dependency installation.

```sh
cmake --preset dev-debug
cmake --build --preset dev-debug --target aerotwin_timing_sensitivity --parallel 2
python3 -m venv /tmp/aerotwin-e2-review-venv
/tmp/aerotwin-e2-review-venv/bin/python -m pip install -e '.[dev,plots]'
MPLCONFIGDIR=/tmp/aerotwin-e2-matplotlib XDG_CACHE_HOME=/tmp/aerotwin-e2-cache \
  /tmp/aerotwin-e2-review-venv/bin/python -m aerotwin_experiments.timing_sensitivity \
  --executable build/dev-debug/examples/timing_sensitivity/aerotwin_timing_sensitivity \
  --output /tmp/aerotwin-e2-results-new --plots
/tmp/aerotwin-e2-review-venv/bin/python -m pytest -p no:cacheprovider
/tmp/aerotwin-e2-review-venv/bin/ruff check .
/tmp/aerotwin-e2-review-venv/bin/ruff format --check .
/tmp/aerotwin-e2-review-venv/bin/python -m pip check
ctest --preset dev-debug
ctest --preset dev-release
git diff --check
```

The output directory must be new. Existing C++ tests require their Debug/Release
builds to exist. Python experiment tests skip with an explicit reason if the
Debug experiment executable is absent. Build it before the focused validation.
Reproducibility here means repeated runs with the same toolchain and seeds.
The standard library's Gaussian noise transformation need not be bit-identical
across different C++ library implementations.

| Check | Observed result |
| --- | --- |
| Full Python suite, including focused experiment tests | PASS, 112 tests, no skips |
| Focused tests within that suite | PASS, 12 tests |
| Existing C++ Debug tests | PASS, 230/230 |
| Existing C++ Release tests | PASS, 230/230 |
| Ruff lint and format | PASS |
| Temporary-environment dependency consistency | PASS |
| Git whitespace check | PASS |
| Deterministic sweep and required artifacts | PASS, 5,880 runs and four plots |
| Docker, DDS, PX4 or operational clock validation | NOT RUN, outside E.2 scope |
| New hosted CI | NOT RUN, changes are uncommitted and no PR requested |

The [focused tests](../../tests/python/test_timing_sensitivity.py) cover zero-error
and common-offset equivalence, deterministic configurations/results and jitter,
drift accumulation, acquisition/delivery separation, ordering rejection,
invalid input and evaluation-truth isolation. Existing core tests cover the
production timestamp and numerical contracts.

## Limitations and prerequisites for E.3

This is a short, GPS-only sensitivity study with one cadence, 20 seeds, shared
independent GPS noise and no timing-dependent noise law. It does not validate
physical clock uncertainty, source completeness, pause/reconnect/reset behavior,
real sensor acquisition epochs, other rates, long horizons, combined error
sources, or hard statistical tail bounds. No multiple-sensor fusion is present.
The paired metrics can hide larger pointwise effects and should not become
operational limits without specifying their aggregation semantics.

Before authorizing E.3, review these results and explicitly define its scope.
Approve the speed/acceleration envelope, covered streams and acquisition
semantics, minimum predictor interval, timing allocation metric, confidence and
coverage requirements, validation horizon and latency deadlines. Specify whether
the proposed budget combines offset, jitter, rate and quantization by worst-case
addition or a justified statistical model. Approve any additional sensitivity
coverage, including combined errors, longer runs and actual paired-stream timing.
Any proposed independent source-clock uncertainty validation needs an agreed
reference and observable acquisition-time evidence. Firmware or runtime changes
require their own authorization. E.2 stopped before E.3. The subsequent
[E.3 analysis](../architecture/runtime-clock-uncertainty.md) and
[E.4 decision](m3-timing-release-gate.md) retain M3.4 as BLOCKED.
