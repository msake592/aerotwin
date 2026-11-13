# Kalman prediction

Include `<aerotwin/estimation/KalmanPredictor.hpp>` and link `aerotwin_core`.
`KalmanPredictor::predict(current, target, Qc)` returns a new
[estimator snapshot](state-estimation-foundations.md) or a typed
`PredictionError`. It uses the validated snapshot, the simulation timestamp
contract, and an explicit process-noise matrix. Its public dependencies exclude
simulator truth, dynamics, scenario execution, and sensor generators.

## Model

The state is ordered `[px, py, pz, vx, vy, vz]ᵀ` in world ENU, with
position in metres and velocity in metres per second. For elapsed seconds
`dt`, the constant-velocity transition is `F = [[I, dt I], [0, I]]`.
Prediction advances position by `dt * v` and preserves velocity. The sparse
evaluation of `F * x` uses a fused multiply-add per position axis.

`Qc` is a symmetric 3×3 continuous-time white-acceleration spectral density
in world ENU, in m²/s³. Its off-diagonals represent cross-axis correlation.
The shared covariance validator supports zero, singular, correlated and
mixed-scale density, with the same symmetry and PSD tolerances as the
estimator state. The model integrates uncertainty without drawing samples.

`Q = [[dt³/3 * Qc, dt²/2 * Qc], [dt²/2 * Qc, dt * Qc]]`

`P_next = F * P_current * Fᵀ + Q`

The implementation evaluates the nonzero transition blocks and mirrors the
computed entries to maintain exact symmetry. For each multi-step entry, a nonzero
coefficient below 2⁻⁹⁵⁰ triggers scaling up by 2¹²⁸ when the largest coefficient
is at most 2⁷⁹⁰. The bound leaves room for the time weights and the sum of terms.
Evaluating the same fused expressions at the larger scale and scaling the result
back once reduces premature subnormal rounding, including when larger terms
cancel. The output retains physical units and is validated by the estimator factory.

## Time and errors

Negative target elapsed time returns `InvalidTargetTimestamp`. A different
epoch returns `EpochMismatch`. A duplicate or backward target in the current
epoch returns `NonForwardTime`. Only after these checks does prediction
subtract elapsed nanoseconds. Both values are nonnegative signed 64-bit
counts and the target is strictly greater, so the difference fits. Conversion
to double-precision seconds follows subtraction. The returned state preserves
the target timestamp exactly. Prediction has no clock or implicit epoch reset.

Invalid density returns `InvalidProcessNoise`. Non-finite arithmetic results
return `NumericalOverflow`, including position overflow. If intermediate
covariance arithmetic overflows, prediction retries those entries using
power-of-two scaling for headroom. A finite result that fails the estimator
covariance contract returns `InvalidPropagatedCovariance`. In particular, a
near-null direction accepted within the initial PSD tolerance can yield a
negative variance after propagation. Prediction rejects that result and
performs no PSD projection. Errors leave the input untouched and return no
partial state.

## Determinism and limits

The operation has no mutable state or RNG. Identical inputs reproduce results
in the same supported floating-point environment. Split and single-step
predictions can differ by rounding. Very long intervals lose nanosecond
resolution in double-precision seconds, and extreme scales or cancellation
can round away small contributions. Entries that exceed the scale-up bound
use ordinary evaluation, and the overflow retry can underflow tiny terms in
affected entries. Final rounding still applies on every path. Near-singular results remain subject to the shared
validation tolerance. Measurement updates and fusion are outside M2.7.
