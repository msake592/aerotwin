# M3.3-E.1 estimator timing error budget analysis

Status: **M3.3-E.1 analysis completed**. E.2 sensitivity validation is recorded
in the [experiment report](../reports/m3-estimator-timing-sensitivity.md).
The 1 s GPS / 10 ms diagnostic IMU intervals, 5 m/s speed, 2 m/s² acceleration,
0.5 ms per-stream registration and 1 ms pairwise skew are approved for continued
prototype experimentation only. They are not approved permanent operational
limits. The remaining accuracy/coverage allocations below are engineering
proposals. M3.4 readiness remains BLOCKED. See
[ADR 0011](../adr/0011-source-time-qualification-and-release-gate.md) for the
E.4 decision. The analysis and original experiment recommendations are retained.

The reviewed main revision is
`9699dee7b3070762d8a3010413a2519466ea12cb`, which merged PR #28 with four
successful hosted checks. Repository inspection found no earlier M3.3-E files,
branches, commits or extra worktrees. Process-name inspection found no active
AeroTwin analysis/build process. Temporary AeroTwin evidence directories from
earlier runs were absent at inspection. The committed M3.3-D results remain
available in the repository. Recovery used the existing source and documents.

## 1. Implemented rates, state and operating assumptions

| Quantity | Current executable behavior |
| --- | --- |
| Simulation/truth tick | 10,000,000 ns, 100 Hz |
| Scenario duration | 400 steps, 401 snapshots from 0 through 4 s |
| GPS acquisition | Default 1,000,000,000 ns, 1 Hz, five fixes |
| Estimator prediction | Four predictions, each 1 s, immediately before GPS fixes at 1–4 s |
| GPS correction | Five corrections at 0–4 s, first uses the initialized prior |
| IMU acquisition | 10 ms, 100 Hz, diagnostic only |
| Initial true ENU velocity | (2, 1, 0.5) m/s, norm approximately 2.2913 m/s |
| True world acceleration | Zero in this scenario |
| Initial estimate | Zero position/velocity, covariance diag(100,100,100,4,4,4) |
| Process spectral density | 0.01 I₃ m²/s³ |
| GPS position covariance | I₃ m², one-metre standard deviation per axis |

The 10 ms simulation tick does not impose a 10 ms estimator prediction cadence.
IMU-only ticks produce no estimates or estimation metrics. The executable has
no runtime rate arguments. The acquisition helper accepts a different GPS
period programmatically, subject to exact sampling deadlines. The core
predictor accepts any strictly positive representable interval within an epoch.
The unit suite includes a one-nanosecond interval and much longer intervals.
There is no universal minimum predictor interval or application timing tolerance.
Python modules provide runtime validation rather than another M2 estimator
configuration. [Acquisition implementation](../../examples/fusion_demo/ScenarioAcquisition.cpp),
[default GPS period](../../examples/fusion_demo/ScenarioAcquisition.hpp),
[estimation loop](../../examples/fusion_demo/FusionEvaluation.cpp),
[predictor tests](../../core/tests/unit/KalmanPredictorTest.cpp)

The estimator state is x = [pᵀ,vᵀ]ᵀ in ENU. For h seconds,

```text
F(h) = [[I, hI], [0, I]]
Q(h) = [[h³D/3, h²D/2], [h²D/2, hD]]
x⁻ = F(h)x
P⁻ = F(h)PF(h)ᵀ + Q(h)
H = [I, 0]
S = H P⁻ Hᵀ + R
K = P⁻ Hᵀ S⁻¹
```

D is continuous white-acceleration spectral density. The mean velocity remains
constant during prediction. D integrates uncertainty and does not supply a
deterministic acceleration bound. The truth integrator also uses constant
linear velocity. Factories enforce numerical validity, while physical speed
and acceleration envelopes remain unspecified. GPS Gaussian noise and filter
estimates have no hard finite speed bound implied by this fixture.
[Prediction model](kalman-prediction.md),
[process density constant](../../examples/fusion_demo/FusionEvaluation.hpp),
[truth integration](kinematic-integrator.md)

## 2. Existing time contracts and error definitions

Core timestamps contain an explicit epoch and signed 64-bit elapsed nanoseconds.
The predictor rejects epoch changes and non-forward targets. The GPS updater
requires exact equality between its prior timestamp and measurement acquisition
timestamp. Sampling rejects missed deadlines. Same-time truth/estimate pairs
are required for metrics. These are structural invariants, while the physical
accuracy of a mapped external acquisition time is a separate requirement.
A physically biased timestamp can satisfy exact software equality.
[Simulation-time ADR](../adr/0004-simulation-time.md),
[predictor checks](../../core/src/estimation/KalmanPredictor.cpp),
[GPS update checks](../../core/src/estimation/GpsKalmanUpdater.cpp),
[sampling deadlines](../../core/src/sensors/SensorSamplingPipeline.cpp)

Let true source-relative event time be t and its mapped label be

```text
t̂_s = t + b_s + ρ_s(t − t_ref) + j_s(t) + q_s(t)
ĥ − h = ρ_s h + j_s(t₂) − j_s(t₁) + q_s(t₂) − q_s(t₁)
```

b is a constant registration offset, ρ a fractional rate error, j variable
timestamp jitter, and q representation/quantization error. A changing offset
adds its change to the second expression.

| Effect | Estimator consequence |
| --- | --- |
| Common constant offset | Cancels from intervals and cross-stream alignment when every participant and epoch origin share it |
| Relative constant offset | Biases alignment to another sensor or truth clock |
| Jitter | Perturbs intervals and association even with zero mean |
| Drift | Changes interval scale and accumulated cross-stream registration |
| Latency | Delays availability. Accurate acquisition timestamps can preserve event-time estimation |
| Cross-sensor acquisition misalignment | Associates observations from different physical instants, independent of simultaneous arrival |

A stable common offset may be arbitrarily large before origin subtraction,
subject to integer representation limits. Independently zeroing each sensor's
first received packet cannot establish a common origin. For independent
zero-mean per-event jitter of standard deviation σ, interval jitter has
standard deviation √2 σ. Correlated jitter instead requires the covariance
term Var(j₂−j₁) = Var(j₂)+Var(j₁)−2 Cov(j₁,j₂). Worst-case bounded endpoint
errors of magnitude E give an interval error at most 2E.

Reception-time stamping converts variable latency into acquisition-time error.
The current updater cannot apply a late measurement to a prior at a different
time, and it implements no rollback/replay. A caller must retain acquisition
times and arrange matching priors in event order. Buffering limits, maximum
waiting time and stale-message recovery require separate application decisions.

## 3. Mathematical sensitivity

### Prediction mean and physical alignment

For the same prior, applying h+e instead of h in the implemented model gives

```text
δp = v e
δv = 0
```

This is exact for the constant-velocity mean model. A wrong elapsed interval
does not directly accelerate this predictor's velocity estimate.

For physical time association under approved speed and acceleration bounds V
and A, Taylor bounds give

```text
||p(t+e) − p(t)|| ≤ V |e| + A e²/2
||v(t+e) − v(t)|| ≤ A |e|
```

V bounds speed at the reference event and A bounds acceleration over the
interval. For a prediction-error bound, also bound the estimated velocity.
These physical bounds do not follow from D. Unmodelled acceleration over an
entire prediction interval h can already cause approximately A h²/2 position
and A h velocity discrepancy. That model error must be assessed separately
from the incremental timing contribution.

### Propagated covariance

Write the prior blocks as Ppp = B₀, Ppv = B₁, Pvv = C. For a fixed prior and
density D, exact real-arithmetic differences from an elapsed-time error e are

```text
δPpp = e(B₁+B₁ᵀ+2hC+h²D) + e²(C+hD) + e³D/3
δPpv = e(C+hD) + e²D/2
δPvv = eD
```

Pvp is the transpose. These expressions show sensitivity of the existing
propagation calculation. They do not automatically account for uncertainty
in the timestamps. Total covariance error depends on the prior blocks,
cross-correlation, interval and density. A bound on Q coefficients alone
cannot bound every entry, norm or eigenvalue of P.

For scalar relative interval error r = e/h, nonzero Qpp, Qpv and Qvv coefficients
change by (1+r)³−1, (1+r)²−1 and r respectively. At h = 1 s and e = 1 ms,
their positive changes are 0.3003001%, 0.2001% and 0.1%.
The first prediction starts after the GPS correction at zero. Its C = 4I,
B₁ = 0 and D = 0.01I. The same 1 ms perturbation changes its diagonal
Ppp by approximately 0.00801401 m², Ppv by 0.004010005 m²/s, and Pvv by
0.00001 m²/s². These are analytical perturbations, with no new benchmark run.
[Covariance propagation](../../core/src/estimation/KalmanPredictor.cpp),
[initialization and event order](../../examples/fusion_demo/FusionEvaluation.cpp)

### Measurement correction and consistency

If a GPS observation acquired at t is tagged t+e and the prior is aligned to
that label, its leading innovation bias is approximately −v e. A bounded
acceleration adds a second-order term. The correction inherits K times this
innovation bias. Velocity may consequently be biased through the position–
velocity cross-gain, even though CV prediction itself preserves velocity.

For zero-mean time jitter independent of the state and GPS noise, a local
linearization adds approximately v vᵀ σ_t² to effective position measurement
covariance. Biased offsets generate systematic innovation error and cannot
be justified solely by increasing measurement noise. With fixed S and bias b,
the expected NIS shifts by bᵀS⁻¹b under the nominal innovation model.

Likewise, uncertain propagation time can add state uncertainty through the
time Jacobian [vᵀ,0ᵀ]ᵀ. Averaging F(h+e)P F(h+e)ᵀ and Q(h+e) contributes
additional terms. Simply adding v vᵀ σ_t² to position covariance omits those
terms. None of these timing-noise corrections is present in the current core,
and this analysis does not authorize adding them.

## 4. Candidate budget for approval

The following numbers are engineering allocations for review. Existing
repository requirements do not prescribe these accuracy limits.

| Proposed assumption or allocation | Candidate |
| --- | --- |
| Scope | Present GPS-aided CV mode, nominal estimator interval 1 s |
| Physical and estimated speed envelope | V ≤ 5 m/s |
| Physical acceleration envelope for local association analysis | A ≤ 2 m/s² |
| Maximum incremental position error allocated to timing | 10 mm |
| Maximum incremental physical velocity misalignment | 10 mm/s |
| Relative acquisition registration error between compared streams | At most 1 ms |
| Residual registration error of each stream to an agreed source epoch | At most 0.5 ms |
| Prediction interval error from two such endpoint errors | At most 1 ms |
| Process-noise coefficient perturbation allocation | At most 1% at h = 1 s |

A 1 ms registration error gives a position bound of 5.001 mm and a physical
velocity bound of 2 mm/s under these envelopes. The exact demo velocity gives
2.2913 mm displacement at 1 ms because its true acceleration is zero.
The 0.5 ms per-stream bound produces a 1 ms worst-case pairwise skew. For
timestamps from one stream, the endpoint bound also limits prediction-interval
error to 1 ms. A common constant origin offset is excluded from these residual
registration errors because it cancels consistently.

For a symmetric endpoint allocation E, use registration skew δ = 2E and
interval perturbation at most 2E. With positive V, A and h, the proposed
allocations imply

```text
E ≤ min(
  [sqrt(V² + 2 A εp) − V] / (2 A),
  εv / (2 A),
  h [(1 + εQ)^(1/3) − 1] / 2
)
```

Here εp = 0.01 m, εv = 0.01 m/s and εQ = 0.01. The three bounds are
approximately 0.9996 ms, 2.5 ms and 1.6611 ms. The proposed E = 0.5 ms
reserves approximately a factor of two against the tightest of these local
allocation limits. For A = 0 use the continuous position limit εp/(2V).
These bounds still depend on the proposed accuracy allocations and motion
envelopes being approved.

The 1 ms interval error meets the proposed 1% Q coefficient allocation at
the current one-second interval. If the same allocation were applied at a
future 10 ms estimator interval, a 1 ms error would change Qpp's coefficient
by 33.1%. The positive relative-error limit for a 1% cubic coefficient
perturbation is approximately 0.0033223. At 10 ms this allows about 33.22 µs
interval error, or 16.61 µs per endpoint under a symmetric worst-case split.
A future faster predictor therefore requires a separate budget.

If a zero-mean relative timing standard deviation were bounded by 1 ms, the
proposed V = 5 m/s envelope would bound the leading effective measurement
covariance contribution's largest eigenvalue by 0.000025 m². This is 0.0025%
of the demo's unit GPS variance. This comparison is an allocation argument.
It does not demonstrate independence, zero mean, consistency or an approved
physical-motion envelope.

All components of the 0.5 ms per-stream allowance must be combined, including
relative offset, jitter, drift and timestamp quantization. A hard bound uses
worst-case addition. Root-sum-square requires justified statistical assumptions.
As an illustrative suballocation, 100 µs of drift over a 10 s validation window
allows 10 ppm residual relative rate error, with the other 400 µs reserved for
remaining terms. That split and the validation window require approval.
Microsecond integer conversion preserves source precision but creates no
sub-microsecond source information.

### Approval and unresolved requirements

Approval is required for the operating envelope, error allocations, covered
streams, minimum prediction interval, confidence level and revalidation window.
The proposed acceleration envelope is an association stress assumption.
Sustained acceleration at that level can exceed the CV model's useful regime
over a one-second interval, independently of timestamp quality.

Before accepting this budget, determine whether limits apply to per-axis error,
3D norm, bias, standard deviation or high-confidence tails. The candidate above
uses 3D norm bounds for physical displacement/velocity and bounded per-stream
registration error. Gaussian noise has unbounded tails, so a statistical
coverage claim needs an explicit confidence criterion and observation horizon.
Specify acceptable RMSE, NIS/NEES degradation and full-P perturbation, with
separate treatment of nearly singular covariance. The physical misalignment allocations do not by themselves bound posterior
velocity error, because the GPS cross-gain and repeated biased updates affect
that error. Also specify source sample
age semantics, cross-sensor association requirements and operational latency
deadlines. These are open requirements.

The M3.3-D 1,140 matched payloads and 1/3 µs maximum median-relative residuals
characterize serializer-offset variation within two captures. They provide no
absolute source-registration, drift, latency or acquisition-alignment bound.
They cannot be compared directly with 0.5 ms as evidence of compliance.
Source sampling completeness and synchronization uncertainty remain open.
[M3.3-D measurements and limitations](../guides/m3-source-wire.md)

## 5. Clock contract implication

Absolute wall-clock synchronization is unnecessary for the current estimator
equations. They require correct elapsed intervals and same-event measurement/
prior association in one agreed simulation epoch. System wall time can remain
a diagnostic field. Rate calibration to seconds remains necessary because
velocity and D use SI units.

A run-relative source-clock contract could satisfy the estimator if all
consumed acquisition times are mapped to the same source origin and rate,
uncertainty is bounded against an approved budget, and restarts invalidate
previous temporal assumptions. Subtract integer origins before converting
small differences to floating-point seconds. Preserve raw source identity,
publication/acquisition times, reception diagnostics and mapping generation.
A publisher change or synchronization-mode change requires revalidation.

Original uORB acquisition time or a separately validated source-time
serialization mode could support this direction. Such a mode has not been
approved or verified by this analysis. Epoch-scaled XRCE timestamps require
their own offset and rate evidence, including pause/reconnection transitions.
The existing exact timestamp checks remain mandatory. An analytical timing
allowance does not relax the GPS updater's equality check, snap observations
to ticks, or enable interpolation/late-update behavior.

## 6. Original E.1 recommendations for M3.3-E.2

This section records the E.1 planning checkpoint, before sensitivity execution.
E.2 subsequently executed its scoped sweep and records its results and limits
in the linked report. The broader recommendations below remain historical
proposals rather than a claim that every listed experiment was implemented.

| Deterministic experiment | Construction and expected discriminator |
| --- | --- |
| Shared origin offset | Shift all raw source labels and their origin together, including a large representable offset, before conversion. Core elapsed labels and relative results should remain invariant |
| Relative GPS offset | Preserve acquired positions/noise and alter mapped acquisition labels relative to truth. Separately record structural API rejection and physical alignment error |
| Interval error | Apply fixed-prior h±e predictions and compare state/P differences with the exact formulas above |
| Jitter | Use fixed alternating and correlated integer perturbations. Evaluate interval differences separately from absolute per-event jitter |
| Drift | Apply a known integer/rational clock-rate perturbation over a declared horizon. Keep quantization error explicit |
| Latency | Delay delivery while preserving acquisition timestamps and event order. Offline results should remain invariant. A separate deliberately reordered stream should exercise existing rejection |
| Cross-stream skew | Preserve a common epoch and offset GPS versus diagnostic IMU acquisition labels. Report alignment effects separately from current estimator outputs |
| Model error control | Compare zero-acceleration truth with a separately declared deterministic accelerating reference in the experiment layer. Isolate timing effects from CV model mismatch |
| Cadence dependence | Retain the 1 s baseline and explicitly investigate a prospective 10 ms prediction schedule as a different configuration |
| Boundary failures | Exercise duplicate/backward labels, epoch changes and overflow. Separate valid analytical timing perturbations from invalid contract inputs |

Use the existing seeded GPS/IMU observations consistently between reference
and perturbed runs. Report posterior position/velocity errors, RMSE, prior
innovation/NIS, posterior NEES, P block differences and source-time errors.
Compare analytical bounds at the candidate limits and at clearly labelled
stress levels. Keep true acquisition time, assigned label and intended state event time
separate in every trial. For an interval-error trial, compare propagation
against the original true target. For a sensor-skew trial, retain z(t) while
associating it with a prior at t+e, and score that prior/posterior against
reference truth at t+e. Preserve truth at t as acquisition provenance.
Interpolation or analytic reference evaluation belongs in the experiment
layer. The stock demo's exact tick checks will reject simple relabelling.
A future harness must declare its association and scoring rules explicitly,
without silently relabelling truth or altering measurement values.

Use identical priors for the local derivative checks. End-to-end runs should
retain their evolving priors and identify that additional dependency.
Stochastic-noise claims require a separately approved seed ensemble and
statistical coverage plan. M3.3-E.2 should test whether the candidate allocations
are useful, rather than treat them as approved acceptance thresholds.
