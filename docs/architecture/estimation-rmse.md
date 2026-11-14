# Estimation RMSE

Include `<aerotwin/validation/EstimationMetrics.hpp>` and link `aerotwin_core`.
`computeRmse` accepts a read-only span of `EstimateTruthPair` snapshots and
returns an owned `RmseResult` or `RmseFailure`. Truth enters through the
validation module. Estimation and sensor code have no dependency on this API.

## Metric and alignment

For each pair, subtract the truth position and linear velocity from the
corresponding estimate, in world ENU. For each of these error vectors, the
per-axis result is `sqrt(sum(e_axis²) / N)` and the 3D result is
`sqrt(sum(||e||₂²) / N)`. Both use the number of pairs `N` as denominator.
The position results are in metres and velocity results in metres per second.
The 3D result is the Euclidean norm of the per-axis RMSE vector up to rounding.

Each pair must match exactly in epoch and elapsed integer nanoseconds.
A batch uses a single epoch. The caller ensures the snapshots come from the
same run and estimator phase. The function visits pairs in supplied order
and weights each equally, including repeated timestamps. The caller controls
sampling frequency and duplicate handling.

## Arithmetic and failures

Each axis and each 3D metric accumulate squares relative to a power-of-two
scale, with the exponent stored separately. For opposite-sign coordinates
near the top of the double range, subtraction uses halved operands and the
accumulator restores the exponent. Division by the sample count and the
square root occur before restoring the result to physical units. This permits
a finite mean when an individual error or its square exceeds the double range.

Validated snapshots supply finite coordinates. Exact zero errors return zero.
Representable subnormal results are supported. A result that overflows or a
positive aggregate that rounds to zero returns `NumericalFailure`. Coordinate
subtraction and accumulation still use double-precision significands, so
rounding can lose contributions small relative to the dominant terms.
Reproducibility assumes the same input order and floating-point environment.

An empty span returns `EmptySamples`. The first invalid pair returns
`EpochMismatch` or `TimestampMismatch` with its zero-based index, checking
epoch first. Batch-wide numerical failures and empty input have no index.
Every failure returns without a partial aggregate.

## Normalized Estimation Error Squared

`computeNees(estimate, truth, components)` requires the same exact epoch and
elapsed nanoseconds as RMSE. The caller supplies snapshots from the same run.
It returns a dimensionless `ConsistencyScore` or an `EvaluationError`.
The residual is estimate minus truth in world ENU, with position in metres
and velocity in metres per second.

For the selected covariance `A` and residual `e`, NEES is `eᵀ A⁻¹ e`.
`FullState` uses the full covariance in position-then-velocity order and
reports six degrees of freedom. `Position` and `Velocity` each use their
own 3×3 marginal covariance block and report three degrees of freedom.

The shared covariance validator checks the selected matrix.
`InvalidCovariance` indicates failure of that policy. NEES also requires
positive variances and usable numerical rank. With `A = D C D`, `D` holds
standard deviations and `C` is the correlation matrix. The smallest
eigenvalue of `C` must exceed `dimension * epsilon * largest_eigenvalue`,
where epsilon is for double precision. A failed rank check or factorization
returns `UnsolvableCovariance`, including for zero residual. The criterion
depends on correlation rather than absolute variance. The evaluator uses
neither regularization nor a pseudoinverse.

A rounded standard deviation can erase variance information. For example,
`sqrt(1 + 2⁻⁵²)` rounds to one in double precision and can cause a false
overflow when computing a score near the largest finite double. To preserve
that variance information, the evaluator equilibrates `A` using a diagonal
matrix `B` of powers of two derived from the variance exponents. It factors
`Π (B⁻¹ A B⁻¹) Πᵀ = L T Lᵀ` with pivoted LDLᵀ, then solves
`L w = Π B⁻¹ e` and sums `w_i² / T_ii`. For diagonal covariance this
reduces to summing `e_i² / A_ii`. For correlated covariance the triangular
solve retains the coupling between axes. The evaluator forms no inverse.

Binary exponents carry scale through residual subtraction, right-hand-side
normalization, squaring and division by the pivots. A rounded square can lose enough information to change a subnormal rounding
decision. The evaluator squares the normalized residual fraction and uses
`std::fma` to recover the product remainder with one rounding. Normalization
keeps the product and any nonzero square remainder in the normal range.
It also uses a fused subtraction to recover the residual of division by the
pivot, adds the square remainder, and divides that combined residual by the
pivot. Both the quotient and its low-order correction enter the compensated
sum at the same binary exponent.

Sequential addition can round several small
terms upward separately and cause false overflow. The evaluator aligns the
contributions and their signed corrections at a common exponent and uses Neumaier compensation to retain
addition remainders. With round-to-nearest and gradual underflow, the
larger-first subtraction recovers the remainder of each aligned addition.
The aligned terms are bounded, so those additions cannot overflow. The
evaluator adds the compensation before restoring the final exponent.

Alignment can round exceptionally small terms, and the compensation sum
itself has double precision. Factorization, term calculation, and final
subnormal rounding also limit accuracy. `NumericalFailure`
covers a normalized nonzero right-hand-side component that becomes subnormal
or zero, a non-finite solve or output, and a positive score rounded to zero.
Representable subnormal scores are supported. The factorization, solve and
significands use double precision, so conditioning and rounding still limit
accuracy. Truth stays within evaluation.

Near the subnormal boundary, rounding the sum and compensation before
restoring the exponent can erase the remainder that distinguishes an
exact midpoint from a value just above it. The evaluator scales both
parts into units of `denorm_min()`, separates whole and fractional
units, and uses the fractional addition remainder to resolve midpoints.
Exact ties round to an even number of units. It then restores the
physical exponent. A positive score rounded to zero still returns
`NumericalFailure`. This assumes binary double precision,
round-to-nearest-even and gradual underflow. Earlier term rounding and
the finite precision of the compensation still limit accuracy.

## GPS Normalized Innovation Squared

`computeGpsNis(prior, measurement)` takes the estimator snapshot immediately
before GPS correction. The caller selects that snapshot. A posterior at the
same timestamp passes the time check but answers a different question.
The prior and GPS acquisition must match exactly in epoch and elapsed
nanoseconds, with epoch checked first.

GPS observes world ENU position in metres with `H = [I₃, 0]`. The innovation
is `y = z − p_prior`, and its covariance is `S = P_prior,pp + R` in m².
The dimensionless score `yᵀ S⁻¹ y` reports three degrees of freedom. NEES
compares estimate error against truth and state covariance. GPS NIS
compares the observation against the prior using innovation covariance
and requires no truth.

Like GPS correction, NIS returns `NumericalFailure` if the raw subtraction
or covariance addition is non-finite. It uses the same covariance validation
and three-dimensional correlation rank cutoff. Singular or numerically
rank-deficient innovation covariance returns `UnsolvableCovariance` even for
zero innovation. Invalid covariance returns `InvalidCovariance`.

NIS shares the NEES evaluator's pivoted LDLᵀ solve, compensated arithmetic,
and final score conversion. GPS correction uses correlation-space Cholesky
for its gain solve. Near numerical boundaries, factorization and range
checks can differ. A valid score does not imply that the later correction
or posterior covariance calculation will succeed. The NEES arithmetic
limits above also apply to NIS.

## Deterministic integration scenario

M2.9.5 exercises prediction, correction, and all three metrics in a test
fixture. Epoch 7 starts at 0 ns with a zero estimate and identity
covariance. Each prediction advances by exactly 1,000,000,000 ns with
zero process noise. The fixture supplies analytical GPS observations in ENU
and keeps truth on the evaluation path. Truth position is `(t, 0.5 + t/2, 0)`
in metres for time `t` in seconds. Truth velocity is `(1, 0.5, 0)` in m/s.

| Time | GPS position (m) | GPS variance per axis (m²) | Prior NIS | Posterior NEES (full / position / velocity) |
| --- | --- | ---: | ---: | --- |
| 1 s | (2, 0, 0) | 2 | 1 | 3/2, 1, 2/3 |
| 2 s | (3.5, 0, 0) | 11/4 | 8/11 | 53/22, 20/11, 244/451 |

For each axis, the first prediction gives position variance 2 and
position–velocity covariance 1. Adding GPS variance 2 gives innovation
variance 4. The x innovation is 2, so NIS is 1 and the position and velocity
gains are 1/2 and 1/4. The posterior x position and velocity are 1 m and
1/2 m/s. Its per-axis position–velocity covariance block has diagonal
entries 1 and 3/4 with cross term 1/2. Scoring the first observation
against this posterior instead would give NIS 1/3.

The second prediction uses the first posterior, giving x position 3/2 m
and per-axis position variance 11/4. The second innovation is 2 m along x
with variance 11/2 m². The position and velocity gains are 1/2 and 5/22.
The posterior x position and velocity are 5/2 m and 21/22 m/s. The per-axis
posterior covariance block has diagonal entries 11/8 and 41/88 with cross
term 5/8. The full-state NEES uses each complete 2×2 block, including the
cross term. The marginal scores use only their respective variances.
NIS has three degrees of freedom. Full-state NEES has six and each
marginal has three.

The two posterior position errors are `(0, −1, 0)` and `(1/2, −3/2, 0)`
in metres. The velocity errors are `(−1/2, −1/2, 0)` and `(−1/22, −1/2, 0)`
in m/s. The per-axis position RMSE is `(√(1/8), √(13/8), 0)` m and
the 3D position RMSE is `√(7/4)` m. Per-axis velocity RMSE is
`(√(61/484), 1/2, 0)` m/s and 3D velocity RMSE is `√(91/242)` m/s.

Run the fixture with `ctest --preset dev-debug -R EstimationEvaluationTest`
or the corresponding release preset. The fixture checks exact replay within
each build and explicit errors for alignment, singular covariance, score
overflow, and an invalid RMSE batch. Fixed observations remove RNG from
the analytical expectations. The fixture checks metric integration and
phase selection. Statistical consistency and end-to-end fusion remain
outside its scope. M2.10 will provide the fusion demo.
