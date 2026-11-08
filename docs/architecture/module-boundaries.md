# Intended module boundaries

The standalone core implements math primitives, the simulation clock, the
[vehicle truth snapshot](../adr/0009-vehicle-state.md), and
[constant-velocity propagation](kinematic-integrator.md). M1.8 adds
[scenario execution and truth export](scenario-telemetry.md). The Python experiment
package is a scaffold. The table also describes planned module responsibilities.

| Location | Responsibility | Boundary |
| --- | --- | --- |
| `core/.../common` | Shared data contracts and basic utilities | No middleware types |
| `core/.../math` | Numerical and geometric primitives | Explicit units and frames |
| `core/.../simulation` | Simulation clock, vehicle ground truth, kinematic propagation, and in-memory scenario execution | Future sensor models may derive measurements from truth. Estimators receive measurements through separate contracts. |
| `core/.../telemetry` | Versioned truth CSV and run metadata export | File I/O consumes completed scenario results |
| `core/.../sensors` | Validated GPS and IMU contracts and generation interface | Measurement headers are independent of truth state. Sensor models may borrow truth explicitly. |
| `core/.../estimation` | Custom state estimation and uncertainty | Independent of PX4's internal estimator |
| `core/.../autonomy` | Decisions and navigation | Consume estimates through explicit interfaces |
| `core/.../validation` | Metrics and comparison logic | Truth available for evaluation, not implicit estimator input |
| `simulation/` | Worlds, models, scenarios, configuration, launch assets | Simulator-specific content stays outside core |
| `ros2_ws/src/` | ROS 2 adapters and integration | Translate messages at the boundary |
| `experiments/` | Python orchestration, configuration, analysis | Record seeds, versions, and run metadata |
| `visualization/` | RViz configuration and visual assets | Observe results without controlling core dependencies |
| `tools/` | Setup, scripts, profiling utilities | No hidden runtime requirements |
| `tests/` | System and regression checks | Exercise explicit acceptance criteria |
| `data/` | Small sample data and schemas | Generated datasets belong in ignored locations |
| `outputs/` | Generated logs, telemetry, experiments, reports | Ignored, never a source dependency |

Core unit and integration tests belong in `core/tests/`. Public core headers will live under `core/include/aerotwin/`, with implementations under `core/src/`.

Adapters may depend on the core; the core must not depend on adapters. Simulation, estimation, autonomy, and validation remain distinct components. Specific interfaces and dependency versions will be designed in later steps.
