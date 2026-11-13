# GPS measurement update

`GpsKalmanUpdater::update(prior, measurement)` takes validated
`EstimatorState` and `GpsMeasurement` values and returns a new estimate or a
`GpsUpdateError`. Include `<aerotwin/estimation/GpsKalmanUpdater.hpp>` and link
`aerotwin_core`. The updater depends on the measurement contract and simulation
time. It has no dependencies on ground truth, dynamics, or sensor generators.

## Model and solve

The state is `[px, py, pz, vx, vy, vz]ᵀ` in world ENU. Position is in metres,
velocity in metres per second, and the GPS position covariance `R` in m².
The fixed-size implementation uses `H = [I₃, 0]` implicitly through block
selection. The innovation is `y = z − Hx` and its covariance is
`S = HPHᵀ + R`.

The shared covariance validator checks and canonicalizes `S`. With positive
diagonal variances, write `S = D C D` where `D` holds their square roots and
`C` is the correlation matrix. Require the smallest eigenvalue of `C` to
exceed three double-precision epsilons times its largest eigenvalue. This
rejects directions near the resolution of a three-dimensional double-precision
solve and leaves the criterion independent of axis units and scales.
Cholesky (Eigen LLT) solves `C (D Kᵀ) = D⁻¹ H P`, followed by diagonal
division to recover the Kalman gain. Each right-hand side is normalized by
a power of two before the solve. The exponent is restored during gain recovery.
A nonzero right-hand-side coefficient that would become subnormal or zero
during normalization produces `NumericalFailure`. The implementation forms no inverse and
applies no regularization. Singular input covariances are supported when
their sum yields a usable innovation system.

Forming the innovation and then adding the correction to a large prior can
round away a small observation. For position, the updater evaluates
`R S⁻¹ p + Ppp S⁻¹ z`. Since `S = Ppp + R`, right multiplication by
`S⁻¹` gives `I − Ppp S⁻¹ = R S⁻¹`. The order matters for correlated
covariances. The updater solves for the transpose of each weight with the
same Cholesky factor and retains its binary exponent through the products.
The independently solved prior weight also supplies the position block of
`I − KH` in Joseph form. With all-zero `R` and usable `S`, the position
weights are exactly zero and identity.

Velocity evaluates `v + K_v (z − p)` per measurement axis. Subtracting
position from observation before combining axes cancels unchanged axes before
they can mask smaller corrections. Each difference retains its rounding
remainder, so an observation lost in the leading difference can still
contribute when the prior velocity cancels that leading term. The updater
multiplies both parts by the exponent-aware gain and retains addition
remainders while summing. Gain and product rounding still limit precision.
A non-finite raw innovation or final output returns `NumericalFailure`.
Exact zero innovation preserves the prior state.

## Covariance and numerical limits

The updater evaluates Joseph form,
`P_post = (I − KH) P (I − KH)ᵀ + K R Kᵀ`, computing one triangle, mirroring it exactly, and validating the posterior
through the estimator factory. Both state correction and Joseph form use
the same gain represented by double-precision fractions and separate binary
exponents. Keeping the exponent through multiplication allows a gain below
the double range to contribute to a representable result. Each covariance
entry is converted back to physical-unit doubles after adding both Joseph
terms. This delays subnormal rounding even when other axes have much larger
variances.

The wider exponent range preserves the equations without increasing
significand precision. Strong cancellation can still erase small contributions,
and near the rank threshold the solve may amplify roundoff. Outputs outside
the finite double range produce `NumericalFailure`. Final covariance validation
uses the established tolerance without clamping eigenvalues.

## Time and errors

The prior and observation must have identical epoch and elapsed nanoseconds.
Both input factories enforce nonnegative elapsed time. The output preserves
the prior timestamp exactly. Callers perform any prediction explicitly.
The updater keeps no measurement history, so the caller must prevent
reapplying the same observation to a posterior.

- `EpochMismatch` and `TimestampMismatch` indicate incompatible timestamps.
- `InvalidInnovationCovariance` indicates that finite `S` failed covariance validation.
- `UnsolvableInnovationCovariance` covers zero variance, deficient numerical rank, or failed Cholesky factorization.
- `NumericalFailure` covers non-finite arithmetic.
- `InvalidPosteriorCovariance` indicates that the finite Joseph result failed validation.

Identical inputs in the same floating-point environment yield deterministic
results. All paths preserve the inputs and return a complete estimate or an error.
