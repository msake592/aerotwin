# Fusion demo scenario and acquisition

The example-local `acquireScenario()` runs the existing scenario runner
and preserves its owned trajectory. It submits each truth snapshot exactly
once to the sensor sampling pipeline. Batch index matches truth index.
The result owns the original measurement slots and both acquisition counts.

The trajectory starts at epoch 0 and 0 ns. It advances for 400 steps of
10,000,000 ns, yielding 401 snapshots through 4,000,000,000 ns inclusive.
Truth starts at ENU position `(10, -5, 2)` m with constant ENU velocity
`(2, 1, 0.5)` m/s and identity body-to-world attitude. Body angular
velocity and world linear acceleration are zero. At elapsed time `t` in
seconds, truth position is `(10 + 2t, -5 + t, 2 + 0.5t)` m.

| Sensor | Acquisition | Covariance and seed |
| --- | --- | --- |
| GPS | 1 Hz from 0 through 4 s, five observations | ENU position covariance `I₃` m², seed 42 |
| IMU | 100 Hz at every truth tick, 401 observations | FLU specific force covariance `0.01 I₃` (m/s²)², seed 43. FLU angular rate covariance `0.0001 I₃` (rad/s)², seed 44. |

The runner constructs timestamp-matched IMU kinematics with zero ENU
linear acceleration. With default ENU gravity `(0, 0, -9.80665)` m/s² and
identity attitude, the noiseless FLU reference is `(0, 0, 9.80665)` m/s²
and zero angular rate. The owned history contains the noisy observations.

The pipeline emits GPS before IMU at all five ties. Other ticks contain
only IMU. The runner checks batch order and acquisition timestamps
against the truth tick and keeps the emitted slots intact.
A nonpositive GPS period fails configuration. A deadline between
truth ticks produces the pipeline's original sampling error at the
tick where it detects the miss. Failures return no acquisition history.

Fresh runs use fresh sensor streams and the same fixed seeds.
Exact replay applies within the same executable and floating-point
environment. The sensor noise contract allows rounding differences
across builds and platforms.

## Estimation and evaluation

The example-local `evaluateFusion()` takes ownership of the acquisition history.
Each GPS event stores its source tick index, the prior, the posterior and aligned
evaluation truth, and the consistency scores. The index resolves to the
original GPS slot in the owned history. Original IMU observations retain their
timestamps, sensor IDs, values, and covariance for diagnostics.

The estimator starts at epoch 0 and 0 ns with zero ENU position and velocity
and covariance `diag(100, 100, 100, 4, 4, 4)` in the estimator's state units.
Prediction uses continuous white-acceleration spectral density `0.01 I₃`
in world ENU (m²/s³). The GPS fix at 0 ns uses the initialized prior
because the predictor requires a strictly forward interval. Each of the four
later fixes, at 1, 2, 3, and 4 seconds, starts from the previous GPS posterior
and predicts to the exact acquisition timestamp. All five fixes are corrected.

At each GPS acquisition the runner computes NIS from the prior and the
sampled GPS observation, then applies GPS correction. Full-state, position,
and velocity NEES use the posterior and same-time evaluation truth. Batch
RMSE uses the five GPS posterior/truth pairs with equal weight. IMU-only
ticks contribute no estimates or metric samples.

Integer epoch and nanosecond comparisons validate each measurement against
the original acquisition tick. The predictor, updater, and evaluation APIs
enforce their own timestamp contracts. No interpolation is used. Truth
position and velocity enter only acquisition and truth-based scoring.
The measurement-driven update receives only the previous estimate and
sampled GPS. IMU values do not influence prediction, correction, or metrics.
This demo performs GPS-aided constant-velocity estimation.

Fresh runs replay exactly within the same executable and floating-point
environment for the same acquisition and evaluation inputs. The runner
returns either the complete evaluation or a staged failure. Failures carry
the original typed error where available, the relevant timestamp and sensor
metadata, and the RMSE sample index when supplied by the metric API.

## CSV v1 report

Build and run the example from the project root:

```sh
cmake --preset dev-debug
cmake --build --preset dev-debug --target aerotwin_fusion_demo
build/dev-debug/examples/fusion_demo/aerotwin_fusion_demo > fusion.csv
```

The executable takes no arguments and is available in builds configured with
`-DBUILD_TESTING=OFF`. It serializes the completed evaluation to memory
before writing to stdout. Usage, acquisition, evaluation, and serialization
failures exit nonzero with a diagnostic on stderr and empty stdout. When
available, diagnostics include the stage, original typed error, timestamp,
sensor ID, and evaluation sample index. On POSIX, the executable ignores
`SIGPIPE` so a stdout pipe with no reader reaches the stream error check.
A detected stdout write failure exits with status 1 and a diagnostic on
stderr. The destination may already have accepted some report bytes.

Every row has the same 132 columns. The first five are `record_type`,
`schema_version` (always 1), `epoch`, `elapsed_ns`, and `sensor_id`.
Epoch and elapsed nanoseconds are exact integers. Sensor ID is populated on
measurement rows. Fields belonging to other record types are empty.

| Record type | Populated fields after the shared columns |
| --- | --- |
| `configuration` | Scenario ID and seed, step count, clock timestep and sensor periods in ns, GPS and IMU channel seeds, ENU gravity in m/s², isotropic process spectral density in m²/s³, initial truth ENU position in m and velocity in m/s. The timestamp is the start of the run. |
| `gps_evaluation` | Measured ENU position in m and measurement covariance in m², prior and posterior ENU position and velocity with their covariances, prior GPS NIS, and posterior full, position, and velocity NEES. Each dimensionless score has a degrees-of-freedom column. |
| `imu_diagnostic` | Body FLU specific force in m/s² and angular velocity in rad/s with their covariances in (m/s²)² and (rad/s)². IMU measurements are diagnostic only. |
| `summary` | Truth and sensor acquisition counts, prediction and successful update counts, rejected observation count, score counts, RMSE sample count, and per-axis and 3D position and velocity RMSE in m and m/s. The timestamp is the last truth tick. |

Vector columns identify axes with `e,n,u` (ENU) or `f,l,u` (FLU).
Covariance columns give the upper triangle in row order, including the
diagonal, with two axis or state labels in the column name. Reconstruct the
lower triangle by symmetry. Estimator covariance labels use
`pe,pn,pu,ve,vn,vu`. Each covariance entry has units equal to the product
of the corresponding state component units. The GPS row at 0 ns also
reports the configured initial estimator state and covariance in its prior
columns. Attitude is fixed at identity with zero body angular velocity and
zero ENU linear acceleration for this scenario.

After one header and the configuration row, acquisitions are emitted in
integer timestamp and original slot order, with GPS first at every tie.
The default report has 5 GPS rows at 0 through 4 s, 401 IMU rows at 10 ms
intervals, and one summary row. A complete run has zero rejected
observations. The report is built only from a successful evaluation.

Numeric output uses the classic locale and `max_digits10` precision.
Text fields containing a comma, quote, CR, or LF are enclosed in double
quotes, with embedded double quotes doubled. Reports omit wall-clock
timestamps and local paths. Byte-identical replay is scoped to runs of the
same executable in the same floating-point environment.
