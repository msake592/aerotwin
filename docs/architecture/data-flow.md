# Intended data flow

Status: conceptual flow; no connections or runtime behavior exist yet.

1. Experiment tools select scenario configuration, seeds, and run metadata.
2. Gazebo evolves simulated dynamics and produces sensor observations and ground truth on distinct channels.
3. ROS 2 adapters validate and translate observations into explicit core contracts, including units, frame identifiers, and simulation timestamps.
4. The custom estimator consumes observations and produces estimated state and uncertainty independently of PX4's internal estimator.
5. Future autonomy consumes estimated state and uncertainty, producing intent for an explicitly designed flight-control integration boundary.
6. PX4 SITL provides flight control; its internal estimation remains a separate path. The precise Gazebo/PX4 transport and control interfaces will be selected and validated during M3.
7. Validation compares time-aligned estimates and behavior against separately routed ground truth. Truth must not silently enter estimator or autonomy inputs.
8. Logging and visualization observe timestamped results. Python analysis produces metrics and reports in `outputs/`.

Each future contract must declare frame, units, clock domain, validity, and relevant covariance semantics. Timestamp ordering, missing observations, reset behavior, and synchronization policy must be specified before implementing fusion. Wall-clock performance measurements must be distinguished from simulated elapsed time.

See [coordinate frames](coordinate-frames.md) for convention requirements.
