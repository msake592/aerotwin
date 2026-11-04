# ADR 0001: Keep the C++ core independent of ROS 2

Status: Accepted (architectural direction; implementation pending)

## Context

AeroTwin needs numerically reliable algorithms, reproducible tests, and multiple simulation and integration paths. Tying algorithms directly to ROS 2 messages, clocks, executors, or package tooling would make isolated testing and reuse harder. Core development must also remain practical on an Apple Silicon machine without NVIDIA hardware.

## Decision

Keep the C++ core independent of ROS 2. Define explicit core interfaces and data contracts; put ROS 2 message conversion, transport, lifecycle, and clock adaptation in integration packages under `ros2_ws/src/`. Gazebo and PX4 dependencies also stay outside the core. Use standalone core tests and document units, frames, timestamps, and ownership at boundaries.

## Alternatives Considered

- Implement everything inside ROS 2 packages: convenient integration, but couples algorithms and tests to middleware and its environment.
- Use ROS 2 message types throughout a nominally separate library: reduces conversion code, but retains middleware coupling in public contracts.
- Avoid ROS 2 entirely: reduces middleware dependencies, but gives up the intended robotics integration ecosystem.

## Consequences

The core can be built and tested independently once build tooling exists. Integration can evolve without dictating algorithm design. Adapters introduce conversion code and require tests to preserve units, frames, timestamps, and covariance semantics. Contracts need deliberate versioning, and standalone tests do not replace later integration tests.
