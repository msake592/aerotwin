# State estimation foundations

M2.6 introduces an owned estimate snapshot in
`<aerotwin/estimation/EstimatorState.hpp>`. Link `aerotwin_core`.
The initial model uses fixed-size Eigen types with state ordering
`[px, py, pz, vx, vy, vz]ᵀ`. Both vectors use world ENU axis order.
Position is in metres and velocity is in metres per second. Orientation
and IMU biases are outside this model.

The 6×6 covariance represents uncertainty in this same ordering. The
position block is in m², the velocity block in m²/s², and the cross
blocks in m²/s. Covariance stays in physical units in the snapshot.

## Construction and covariance

`EstimatorState::create(vector, covariance, timestamp)` returns a validated
value or a typed error for invalid state coefficients, timestamp, or covariance.
The snapshot owns its inputs and accessors return copies. The estimator header
depends on math and the simulation timestamp contract. Vehicle state, simulation
dynamics, scenario execution, truth trajectories, and sensor generators are
outside this dependency boundary. Future updates can accept the separate
measurement contracts directly.

The covariance policy is shared with the existing 3×3 measurement validator.
Entries must be finite and variances nonnegative. A zero-variance axis must
have zero cross terms. Positive-variance pairs are normalized using the
geometric mean of their variances, computed in mantissa/exponent form to
avoid overflowing products and prematurely rounding subnormal denominators.
The two normalized entries must agree within `1e-12` relative to the larger
of one and their absolute magnitudes. Accepted pairs are stored using the
midpoint in physical units. Correlation magnitudes must be at most
`1 + 1e-12`.

A self-adjoint eigensolver checks the dimensionless correlation matrix.
The smallest eigenvalue must be at least `-1e-12`. Isolated zero-variance
axes use unit placeholders in this calculation. Positive definite,
singular, and zero covariance are supported. The tolerance admits tiny
negative modes near numerical null directions. Construction preserves these
modes and performs no PSD projection. Floating-point rounding affects
decisions close to the tolerance boundary. Downstream algorithms must
choose their own policy for factorization or inversion of singular and
near-singular inputs.

## Initialization and simulation time

The caller supplies the initial estimate, covariance, and exact simulation
timestamp. Any epoch and nonnegative elapsed nanoseconds are valid for
initialization. The snapshot preserves both fields, and its epoch defines
the active run for a future estimator initialized from it. Timestamp
equality compares both fields. Constructing a fresh snapshot and explicitly
replacing the current value is the reinitialization contract. A future
estimator must reject ordinary operations from incompatible epochs and
require explicit reinitialization on an epoch change.

[Kalman prediction](kalman-prediction.md) advances elapsed time strictly within
the active epoch. [GPS correction](gps-measurement-update.md) requires an exact
timestamp match and preserves the estimate timestamp. Both operations check the
epoch. Callers coordinate timing and explicit reinitialization across runs.
