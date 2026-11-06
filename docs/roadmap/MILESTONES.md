# Milestone summary and dependencies

| Milestone | Deliverable focus | Depends on | Status |
| --- | --- | --- | --- |
| M0 — Project Foundation | Structure, build/test/CI, and architecture review | None | Complete |
| M1 — Mathematical & Simulation Foundations | Contracts, tested mathematics, minimal deterministic simulation | M0 | In Progress (M1.3 merged; M1.4 pending review) |
| M2 — Sensor Simulation & State Estimation | Sensor models, custom estimator, uncertainty evaluation | M1 | Not Started |
| M3 — Gazebo, PX4 & ROS 2 Integration | Validated stack and adapters | M1, M2 | Not Started |
| M4 — Uncertainty-Aware Autonomous Navigation | Navigation using estimates and uncertainty | M2, M3 | Not Started |
| M5 — Validation & Robustness Benchmarking | Reproducible fault/disturbance benchmarks | M3, M4 | Not Started |
| M6 — Explainable 3D Visualization | Time-aligned inspection and explanation | M3, M4, M5 | Not Started |
| M7 — Multi-Agent Coordination | Coordinated scenarios and evaluation | M4, M5, M6 | Not Started |

Dependencies are completion gates for the planned sequence, not evidence of implemented functionality. Requirements may be refined through ADRs before implementation. See [ROADMAP.md](ROADMAP.md) for objectives, tasks, deliverables, and definitions of done.
