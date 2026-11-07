# Intended architecture

The standalone M1 core implements deterministic scenario execution and truth
telemetry locally. The middleware, sensing, estimation, and autonomy architecture
below describes future work. See the [M1 data flow](data-flow.md).

AeroTwin will separate a standalone C++ algorithm core from simulation and middleware integration. The core will expose explicit interfaces for mathematical utilities, sensor observations, estimation, autonomy, and validation. It must be testable without ROS 2, Gazebo, or PX4.

Gazebo will supply simulated observations and separately identified ground truth. PX4 SITL will provide flight control. ROS 2 will serve as an integration layer, converting external messages into core data contracts and routing results. The custom AeroTwin estimator will be independent of PX4's internal estimator; initial evaluation does not imply replacing PX4's flight-control estimate.

Python tools will configure and orchestrate reproducible experiments, collect artifacts, and analyze metrics. Visualization will explain truth, estimates, uncertainty, and autonomy decisions without becoming an algorithm dependency.

Core development targets the Apple Silicon development machine and must not require CUDA or NVIDIA hardware. Integration compatibility and versions remain to be validated in a later milestone.

See [module boundaries](module-boundaries.md), [data flow](data-flow.md), [coordinate frames](coordinate-frames.md), and [ADR 0001](../adr/0001-core-independent-of-ros.md).

See the [M0 architecture review](m0-review.md) and [robotics integration feasibility assessment](robotics-integration-feasibility.md). The [Python experiment decision](../adr/0002-python-experiments.md), [frame decision](../adr/0003-coordinate-frames.md), and [simulation-time decision](../adr/0004-simulation-time.md) capture the design contracts for later milestones.
