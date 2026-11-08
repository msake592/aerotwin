# M1 deterministic motion report

M1 establishes a standalone, reproducible C++20 baseline for 3D motion and
versioned simulator truth. M1.8 and M1.9 passed verification and merged in
[PR #12](https://github.com/msake592/aerotwin/pull/12).

## Implemented components

| Step | Component |
| --- | --- |
| M1.1 | Pinned Eigen 3.4.0 integration |
| M1.2 | Math types, units, and validation |
| M1.3 | Checked Hamilton quaternion operations |
| M1.4 | World ENU and body FLU transformations |
| M1.5 | Fixed-step clock with integer nanoseconds and reset epochs |
| M1.6 | Validated, timestamped ground-truth vehicle state |
| M1.7 | Constant-velocity kinematic integrator |
| M1.8 | Scenario validation, fixed-step execution, and versioned CSV truth export |
| M1.9 | Headless motion demo and end-to-end analytical verification |

The caller supplies `ScenarioConfig` and owns the export destination.
`runScenario` owns a fresh `SimulationClock` and delegates each step to the pure
`integrateConstantVelocity` operation. The runner stores the returned
`VehicleState` snapshots in order, including the initial state. The exporter
writes only completed scenario results. See the [scenario and telemetry contract](../architecture/scenario-telemetry.md).

## Reference motion

Position follows `p(t) = p0 + v_world * t` in world ENU. The active Hamilton
attitude follows `q(t) = q0 * exp(0.5 * omega_body * t)`, with the rotation
increment on the right because angular velocity is in body FLU. Position uses
metres, world linear velocity uses metres per second, and body angular velocity
uses radians per second.

The reference starts at `(10, -5, 2)` with identity attitude,
`v_world = (2, 1, 0.5)` and `omega_body = (0, 0, pi/4)`. It advances 100 steps
of 100,000,000 ns and records 101 samples from epoch 0, elapsed 0 to
10,000,000,000 ns. The seed is 42 and has no effect on motion.

The analytical endpoint is `(30, 5, 7)` and a positive Z rotation of
`5*pi/2`, physically equivalent to +90 degrees. The observed endpoint on
Apple Silicon macOS was `(29.999999999999929, 4.9999999999999973, 6.9999999999999822)`
with WXYZ quaternion `(-0.70710678118654735, 0, 0, -0.70710678118654757)`.

The final position error was `7.33e-14 m`, also the maximum over all samples.
Quaternion error used the smaller of the Euclidean coefficient differences
against the analytical quaternion and its negation. Final error was
`3.51e-16` and maximum error was `9.44e-16`. The integration tests use
absolute tolerances of `1e-12 m` for position and `1e-12` for quaternion
coefficient distance. The reference is evaluated analytically from each
timestamp without calling the production integrator.

## Verification

- The M1.8 gate passed 83/83 tests in both Debug and Release before M1.9 work.
- The final suite passed 95/95 in Debug and Release, including stationary,
  straight-line, rotation-in-place, and combined-motion integration cases.
  The combined case verifies exact timestamps, velocities, CSV structure,
  metadata, frame mapping, and byte-identical replay.
- The tests-disabled build registered zero tests and built both examples.
  Both examples ran in all three configurations. Two invocations of the
  same Debug demo produced byte-identical CSV and metadata. A forced file
  write failure returned a nonzero exit code and removed the partial file.
  The empty leaf remained in place. Deterministic export tests covered parent and destination
  replacement, symlink rejection, and cleanup after partial output. The
  100,000-step stationary test returned 100,001 snapshots.
- Pytest passed 3/3. Ruff lint, Ruff format, pip check, relative Markdown file
  links, and `git diff --check` passed locally.

## Limits and next milestone

M1 models kinematics. Aerodynamic forces, motors, gravity-based dynamics,
sensor noise, state estimation, EKF, autonomy, multi-agent coordination, and
real-time visualization are outside this implementation. Repeatability applies
to the same executable and floating-point environment. Step subdivision and
math library differences may affect floating-point results. Export consists
of two files and uses descriptor-relative, best-effort cleanup of owned files.
The directory can remain after a failure. See the [export security assumptions](../architecture/scenario-telemetry.md#schema-version-1).

M2 can derive synthetic observations from timestamped truth and compare
independent estimates against it. The estimator boundary will carry
observations and estimates through separate contracts.
