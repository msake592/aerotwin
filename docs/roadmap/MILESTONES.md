# Milestone summary and dependencies

| Milestone | Deliverable focus | Depends on | Status |
| --- | --- | --- | --- |
| M0 — Project Foundation | Structure, build/test/CI, and architecture review | None | Complete |
| M1 — Mathematical & Simulation Foundations | Contracts, tested mathematics, minimal deterministic simulation | M0 | Complete |
| M2 — Sensor Simulation & State Estimation | Sensor models, custom estimator, uncertainty evaluation | M1 | Complete (seeded GPS-aided baseline, diagnostic IMU) |
| M3 — Gazebo, PX4 & ROS 2 Integration | Validated stack and adapters | M1, M2 | In Progress (M3.3-E engineering package complete; M3.4 production timing gate BLOCKED) |
| M4 — Uncertainty-Aware Autonomous Navigation | Navigation using estimates and uncertainty | M2, M3 | Not Started |
| M5 — Validation & Robustness Benchmarking | Reproducible fault/disturbance benchmarks | M3, M4 | Not Started |
| M6 — Explainable 3D Visualization | Time-aligned inspection and explanation | M3, M4, M5 | Not Started |
| M7 — Multi-Agent Coordination | Coordinated scenarios and evaluation | M4, M5, M6 | Not Started |

Dependencies are completion gates for the planned sequence, not evidence of implemented functionality. Requirements may be refined through ADRs before implementation. See [ROADMAP.md](ROADMAP.md) for objectives, tasks, deliverables, and definitions of done.

## M3 timing package and next gate

| Package | Status | Evidence / remaining scope |
| --- | --- | --- |
| M3.3-E.1 | Complete | Estimator timing analysis and prototype-only allocations |
| M3.3-E.2 | Complete | 5,880 deterministic timing runs, 20 seeds, exact repeated results |
| M3.3-E.3 | Analysis complete | Physical acquisition and offset-association uncertainty unresolved |
| M3.3-E.4 | Review and architecture decision complete | [Release gate](../reports/m3-timing-release-gate.md), [ADR 0011](../adr/0011-source-time-qualification-and-release-gate.md) |
| M3.4 production readiness | BLOCKED | Topic acquisition, numerical uncertainty, coverage, invalidation and stale-data evidence required |
| M3.3-F.1 source-time GPS qualification | Proposed, not started | Option A bounded qualification, requires configuration/coverage approval |

Engineering-package completion does not close the M3 integration milestone.
The 0.5 ms per-stream / 1 ms pairwise values and motion/rate assumptions are
approved for continued prototype experimentation only. No production GPS input
qualifies and no new bridge or firmware modification is approved.
