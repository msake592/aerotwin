# M0 architecture review and M1 readiness

The standalone C++20 target and test-only GoogleTest dependency give mathematics and estimation a small build surface. The installable Python scaffold supports independent experiment development. Simulator assets, ROS 2 adapters, visualization, tests, and generated results have separate locations. These boundaries can be tested incrementally as runtime code appears.

| Area | Risk and next decision |
| --- | --- |
| 3D simulation and digital twins | Simulator axes, origins, and clocks can diverge from the core contract. Begin with deterministic core kinematics in M1 and test adapter alignment and reset behavior when adding external simulation in M3. |
| Sensor simulation, fusion, and uncertainty | Truth can leak into estimator inputs and covariance can be misinterpreted across frames. Keep observations, truth, and estimates distinct and declare covariance layout, frame, units, noise, and seeds as each M2 interface is added. |
| Autonomous navigation | Depending directly on simulator truth or PX4's internal estimate would change what the experiment measures. Define M4 inputs around the selected estimator output and explicit uncertainty and validity. |
| Fault injection and robustness | Unrecorded faults and scenario changes undermine comparisons. Keep experiment configuration and run provenance in Python and make faults explicit at the appropriate simulation or adapter boundary in M5. Keep large outputs in ignored locations. |
| Explainable visualization | Unsynchronized overlays can misrepresent estimator quality. Use timestamped, frame-tagged output and preserve distinct truth, estimate, uncertainty, and decision streams. Keep viewers downstream of the core. |
| Multi-agent coordination | Shared frame origins, timestamp epochs, and agent identity will need explicit contracts. Validate the single-agent path first and scope coordination and transport semantics in M7. |

## M1 readiness checklist

- [x] Standalone build and C++ unit-test entry points support incremental mathematical work.
- [x] Frame, unit, quaternion, and simulation-time conventions are recorded as design decisions.
- [x] Local build and Python audit and hosted CI evidence are recorded in the [roadmap](../roadmap/ROADMAP.md#step-08--architecture-review--m0-readiness).
- [ ] M1 starts after M0 review and merge. First select an Eigen version and integration method, then validate a small standalone C++ target on the existing CI platforms. Add vector and matrix tests before rotations, simulation time, vehicle kinematics, and a deterministic ground-truth trajectory.

There are no technical blockers identified for M1. Non-blocking debt includes unlocked Python development dependencies, rotating CI runner tool versions, and the unexercised minimum CMake version. Deferred decisions include Eigen ownership and exposure in public headers, numerical tolerances and step scheduling in M1, sensor noise and late-data policy in M2, and concrete middleware versions and transport in M3. Add contracts when a milestone first needs them.
