# M3.1 integration readiness and feasibility

Evidence checked 2026-11-14. Upstream `main` was
`56ebbfafa92ed6f71fbb0a85f546e04c0c2bd4d8`, the merge of
[PR #22](https://github.com/msake592/aerotwin/pull/22).
The [PR matrix](https://github.com/msake592/aerotwin/actions/runs/43624720282)
and [post-merge matrix](https://github.com/msake592/aerotwin/actions/runs/43625911819)
each passed C++ Debug on Ubuntu 24.04 and macOS 15, and Python 3.11 and 3.13
on Ubuntu 24.04. This is standalone evidence.

## M2 acceptance audit

The scoped [definition of done](../roadmap/ROADMAP.md#m2--sensor-simulation--state-estimation)
requires synthetic estimation and covariance validation, seeded experiments,
reported errors, and ground-truth isolation.

| Area | Evidence |
| --- | --- |
| Sensor contracts and repeatability | GPS ENU and IMU FLU observations retain identity, timestamps and covariance. Seeded Gaussian, GPS, IMU and pipeline tests cover invalid inputs, copy/reset replay and scheduled sampling. [Measurements](../architecture/measurements.md), [noise](../architecture/gaussian-noise.md), [sampling](../architecture/sensor-sampling.md) |
| Estimator and covariance | Validated 6D state, analytical prediction and GPS correction, exact-time enforcement, invalid covariance, singularity and numerical edge cases. [State](../architecture/state-estimation-foundations.md), [prediction](../architecture/kalman-prediction.md), [GPS update](../architecture/gps-measurement-update.md) |
| Evaluation | Analytical fixtures verify RMSE, posterior full and marginal NEES and prior GPS NIS. Demo differential tests change truth or IMU seeds independently and preserve the required estimates, observations and scores. [Metrics](../architecture/estimation-rmse.md) |
| Executable | Five GPS corrections, four predictions and 401 IMU diagnostics from seeded sampling; CSV v1 has 132 columns, 408 data rows and exact-time ordering. Replay, correlated covariance serialization and typed/output failures are tested. [Demo](../architecture/fusion-demo-acquisition.md) |

M2 meets its scoped acceptance criteria. It provides GPS-aided linear
constant-velocity estimation. IMU-driven propagation, full GPS/IMU fusion,
and nonlinear airframe estimation are absent and deferred. Invalid-input
behavior has deterministic tests; broader degradation campaigns and
cross-run statistical consistency require further evidence. These gaps
do not block M3 host validation.

## Candidate stack compatibility

Upstream sources checked 2026-11-14. PX4 GitHub reports v1.17.0 as the
latest stable release, at `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.
v1.18.0-rc1 is a prerelease. These are candidate selections, pending a
resolved package manifest and execution. [PX4 releases](https://github.com/PX4/PX4-Autopilot/releases)

| Platform or pairing | Upstream status and conditions | AeroTwin result |
| --- | --- | --- |
| Ubuntu 24.04 amd64, Jazzy, Harmonic | Official ROS Tier 1 and Gazebo Harmonic support. Jazzy pairs with Harmonic (gz-sim 8, SDFormat 14). PX4 v1.17 also supports Ionic and Jetty on Noble; select Harmonic explicitly for Jazzy. | Standalone CI passes; robotics **NOT RUN**. |
| Ubuntu 24.04 arm64 VM | Jazzy Tier 1, Harmonic ARM best effort. Confirm packages, PX4 plugins, guest networking and DDS separately. | **NOT RUN**; no reachable guest. |
| Native Apple Silicon macOS | Jazzy REP-2000 lists macOS amd64 Tier 3 source only, with no macOS arm64 tier. Harmonic documents Sequoia arm64 packages and best-effort platform support. PX4 separately documents Apple Silicon development. The full native stack is outside the reference profile. | macOS 26.6.2 standalone passes. Robotics **NOT RUN**. |
| PX4 / ROS DDS | PX4 v1.17 uses a 2.x client. Candidate Agent v2.4.3; Agent 3.x is incompatible with that client. A standalone Agent build must use compatible Fast DDS/Fast CDR. Match firmware message definitions with a verified px4_msgs revision starting at release/1.17. Supported mismatched versioned definitions require translation; avoid it for the baseline. | **NOT RUN**. Agent, overlay and transport unverified. |
| Headless Gazebo | Server-only physics can be tested without GUI or rendered sensors. Offscreen rendering requires a separate OGRE2/EGL and driver check. No CUDA requirement for the asset-free smoke world. | **NOT RUN**; GPU acceleration and VM rendering unverified. |

Primary sources: [REP-2000](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst),
[ROS/Gazebo pairing](https://gazebosim.org/docs/harmonic/ros_installation/),
[Harmonic platforms and versions](https://gazebosim.org/docs/harmonic/install/),
[Harmonic macOS](https://gazebosim.org/docs/harmonic/install_osx/),
[PX4 v1.17 Gazebo](https://docs.px4.io/v1.17/en/sim_gazebo_gz/),
[PX4 macOS](https://docs.px4.io/v1.17/en/dev_setup/dev_env_mac),
[PX4 XRCE DDS and Agent](https://docs.px4.io/v1.17/en/middleware/uxrce_dds),
[tagged PX4 ROS guide](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/docs/en/ros2/user_guide.md),
[px4_msgs release/1.17](https://github.com/PX4/px4_msgs/tree/release/1.17),
and [headless rendering](https://gazebosim.org/api/sim/8/headless_rendering.html).

The candidate [px4_msgs revision](https://github.com/PX4/px4_msgs/tree/86d8239e962f6939e05c3737784f60c02fa884db)
has identical Git blob hashes to the tagged firmware for SensorGps,
SensorCombined and VehicleOdometry. This checks source identity for those
three interfaces. A built overlay and decoded DDS samples remain necessary.

## Local environment and decision

- Apple Silicon arm64, macOS 26.6.2 (25G83), 24 GiB memory, approximately
  130 GiB free. Hypervisor support reported by `sysctl -n kern.hv_support`
  is 1. CMake 4.4.4, Apple Clang 21.0.0, Python 3.14.5, Git 2.54.0,
  GitHub CLI 2.103.3.
- Docker CLI 29.4.1 is present. `docker info` could not reach the daemon
  in the existing `desktop-linux` context. No simulator, VM or container
  was started. No ROS, Gazebo, PX4 or XRCE executable was on PATH. No
  ROS installation under `/opt/ros` or relevant ROS/RMW/DDS/PX4/Gazebo
  environment overrides were found.
- [M0 platform review](../architecture/robotics-integration-feasibility.md)
  remains historical. Its Ubuntu-first direction still applies.
  [Environment guide](../guides/m3-environment.md) gives the commands.

| Executed | Result |
| --- | --- |
| `cmake --preset dev-debug && cmake --build --preset dev-debug -j 5 && ctest --preset dev-debug -j 5` | 230/230 passed |
| `cmake --preset dev-release && cmake --build --preset dev-release -j 5 && ctest --preset dev-release -j 5` | 230/230 passed |
| `pytest -p no:cacheprovider` via the project interpreter | 17/17 passed, including 14 preflight cases. Added the existing editable-install source path only to the Python process because macOS marked its local .pth file hidden. |
| `ruff check --no-cache .`, `ruff format --check --no-cache .` and `python -m pip --no-cache-dir check` | Passed |
| `python3 -B experiments/python/src/aerotwin_experiments/m3_preflight.py --profile macos-dev --json` | Exit 0; core tools pass, robotics runtime SKIP |
| Same script with `--profile linux-integration --smoke --json` | Exit 2; platform BLOCKED, simulator and middleware probes SKIP |
| `git diff --check` and local Markdown path/heading check | Passed |

**M3.2 CONDITIONAL GO** for validating an existing Ubuntu 24.04 amd64 host.
Use arm64 after package checks and the same smoke tests. Coupled integration
and production adapters remain gated on actual package resolution, decoded
DDS samples, repeatable startup/shutdown, and confirmed clock and frame
mapping. See the [adapter gates](../architecture/m3-adapter-boundary.md).
