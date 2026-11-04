# Engineering instructions for coding agents

## Architecture

- Keep the C++ core independent of ROS 2; middleware belongs in adapters.
- Separate simulation, estimation, autonomy, and validation.
- Prefer modular components with explicit interfaces and clear ownership.
- Avoid unnecessary dependencies and document justified additions.

## C++

- Prefer modern C++ with RAII, explicit ownership, and const correctness.
- Avoid raw owning pointers.
- Prioritize numerical correctness over premature optimization.

## Simulation

- Distinguish ground truth from estimated state in interfaces and outputs.
- Explicitly document coordinate frames and units.
- Use simulation timestamps consistently; specify clock and reset behavior.
- Avoid mixing coordinate conventions.
- Document sensor noise assumptions and reproducibility controls.

## Testing

- Add relevant tests for new functionality.
- Never remove failing tests merely to make CI pass.
- Report tests that could not be executed and why.
- Do not fabricate benchmark results.

## Scope control

- Implement only the requested roadmap step.
- Do not introduce major frameworks without justification.
- Do not add Unity, Isaac Lab, or reinforcement learning at this stage.
- Do not implement future milestones prematurely.
- Core development must not require NVIDIA-specific hardware.

## Documentation

- Keep README and roadmap status accurate; distinguish planned and implemented behavior.
- Record major architectural decisions in docs/adr/.
- Explain non-obvious engineering choices.
