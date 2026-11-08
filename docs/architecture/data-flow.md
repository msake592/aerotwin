# Data flow

## Standalone M1 pipeline

```text
ScenarioConfig -> runScenario -> ScenarioResult -> exportTruthTelemetry
                       |               |                 |
                       |               |                 +-> CSV and metadata
                       |               +-> ordered VehicleState snapshots
                       +-> owned SimulationClock
                       +-> integrateConstantVelocity
```

The caller owns the configuration and destination. The runner owns a clock and
calls the pure kinematic integrator to produce each validated truth snapshot.
The exporter reads the result without mutating it and owns file I/O. See the
[scenario and telemetry contract](scenario-telemetry.md).

## Sensor contract boundary

M2.1 defines [GPS and IMU measurements](measurements.md). The [GPS generator](gps-simulation.md)
borrows truth at a supplied acquisition timestamp and returns an owned
observation. Estimator inputs carry observations independently of truth.
Scheduling, delivery, and estimation remain planned.

## Planned integration flow

1. Experiment tools select scenario configuration, seeds, and run metadata.
2. Gazebo evolves simulated dynamics and produces sensor observations and ground truth on distinct channels.
3. ROS 2 adapters validate and translate observations into explicit core contracts, including units, frame identifiers, and simulation timestamps.
4. The custom estimator consumes observations and produces estimated state and uncertainty independently of PX4's internal estimator.
5. Future autonomy consumes estimated state and uncertainty, producing intent for an explicitly designed flight-control integration boundary.
6. PX4 SITL provides flight control; its internal estimation remains a separate path. The precise Gazebo/PX4 transport and control interfaces will be selected and validated during M3.
7. Validation compares time-aligned estimates and behavior against separately routed ground truth. Truth must not silently enter estimator or autonomy inputs.
8. Logging and visualization observe timestamped results. Python analysis produces metrics and reports in `outputs/`.

Each future contract must declare frame, units, clock domain, validity, and relevant covariance semantics. Timestamp ordering, missing observations, reset behavior, and synchronization policy must be specified before implementing fusion. Wall-clock performance measurements must be distinguished from simulated elapsed time.

See [coordinate frames](coordinate-frames.md) and [simulation time](../adr/0004-simulation-time.md) for convention requirements.
