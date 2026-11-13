# Development roadmap

Steps 0.1–0.6 are implemented: repository foundation documentation, a minimal standalone C++20 build with a version-reporting example, optional GoogleTest/CTest unit testing, and the Python experiment package with pytest and Ruff. Step 0.7 adds verified GitHub Actions CI. Step 0.8 records the architecture review and M1 readiness. M1.5 implemented the deterministic simulation clock. M1.6 implemented the validated ground-truth vehicle state model. M1.7 implemented constant-velocity propagation. M1.8 and M1.9 added deterministic scenario execution, truth telemetry, and a headless demo. M2.1 adds validated sensor measurement contracts. Estimation and autonomy remain planned. Milestone completion requires evidence against its definition of done.

## M0 — Project Foundation

**Status:** Complete

**Objective:** Establish a documented and reviewable engineering foundation.

**Main tasks:** Create the repository structure and initial documentation; establish engineering guidance and an ADR; establish the standalone C++20 build; establish optional C++ unit testing and the Python development environment; configure and verify CI.

**Deliverables:** Repository skeleton, README, contribution guidance, architecture documents, roadmap, license, formatting and ignore rules, CMake targets and presets, a linked version-reporting example, discovered GoogleTest unit tests, and an editable Python experiment package with pytest and Ruff configuration, plus a GitHub Actions CI workflow.

**Definition of done:** Initial files and links are verified, and remaining foundation setup/build/test/CI work is explicitly scoped and validated before M0 closes.

### Steps 0.1–0.3 checklist

- [x] Repository structure exists and is verified.
- [x] Initial project and architecture documentation exists and is verified.
- [x] Engineering guidance, license, formatting, ignore rules, and PR template are verified.

### Step 0.4 — CMake & C++ Build System

- [x] CMake 3.20+ configuration builds the standalone `aerotwin_core` library with C++20 and no compiler extensions.
- [x] Public version header and compiled implementation link into `aerotwin_basic_core`.
- [x] Debug and Release presets use separate ignored directories under `build/`.
- [x] Both configurations build successfully and both examples print `AeroTwin version: 0.1.0-dev` with exit status 0.
- [x] Required tools, build commands, run commands, and generator alternatives are documented in the [README](../../README.md#getting-started).

**Verification:** Apple Silicon macOS, AppleClang 21.0.0, CMake 4.4.4, Unix Makefiles. Executed `cmake --preset dev-debug`, `cmake --build --preset dev-debug`, `cmake --preset dev-release`, `cmake --build --preset dev-release`, and both generated executables. No robotics or third-party library dependencies were added. Other platforms and the minimum CMake version have not yet been exercised.

### Step 0.5 — GoogleTest & CTest Integration

- [x] Include CTest and gate all test configuration and FetchContent behind `BUILD_TESTING`.
- [x] Pin GoogleTest v1.18.0 with archive SHA-256 verification; retain downloaded sources and artifacts only under ignored build directories.
- [x] Privately link tests to `aerotwin_core` and `GTest::gtest_main`; automatically register cases with CTest.
- [x] Verify non-null, non-empty, and expected development version behavior.
- [x] Debug: configure/build succeeded; CTest passed 3/3 tests.
- [x] Release: configure/build succeeded; CTest passed 3/3 tests.
- [x] Testing disabled: configure/build succeeded; no GoogleTest download/configuration or dependency directory; example exited successfully.
- [x] Preserve Debug/Release configure/build presets and add matching test presets.
- [x] Document test commands, GoogleTest versus CTest, and the testing-disabled build.

**Verification:** Apple Silicon macOS, AppleClang 21.0.0, CMake/CTest 4.4.4, Unix Makefiles. Ran `cmake --preset dev-debug`, `cmake --build --preset dev-debug`, `ctest --preset dev-debug`, and equivalent Release commands. Ran `cmake -S . -B build/no-tests -DBUILD_TESTING=OFF` and `cmake --build build/no-tests`. All three configurations' examples printed `AeroTwin version: 0.1.0-dev` and exited with status 0. The initial sandboxed dependency fetch failed DNS resolution; retrying with approved network access succeeded. Other platforms and CMake 3.20 have not been exercised. Integration tests remain intentionally deferred.

### Step 0.6 — Python Development Environment & pytest

- [x] Root `pyproject.toml` declares Python 3.11+, a lightweight setuptools backend, no runtime dependencies, and a pytest/Ruff development extra.
- [x] Preserve existing experiment directories and place Python sources under `experiments/python/src/aerotwin_experiments/`.
- [x] Add only a standard-library package identity function; no duplicate C++ version function, algorithms, bindings, or robotics dependencies.
- [x] Create local `.venv/`, upgrade pip, and install the package in editable mode with development dependencies.
- [x] Verify imports, expected identity, and editable source/metadata with three passing pytest cases.
- [x] Verify pytest from the repository root and a different working directory inside the repository, without `PYTHONPATH` changes.
- [x] Pass Ruff linting and formatting checks with Python 3.11 target syntax and 100-character formatting.
- [x] Reconfigure/build C++ Debug and pass all three existing CTest cases without C++ changes.
- [x] Verify generated environment, metadata, caches, and build artifacts remain ignored and document setup and responsibilities.

**Verification:** Apple Silicon macOS; Python 3.14.5, pip 26.3, pytest 9.2.0, Ruff 0.17.3. Created `.venv/` using `python3 -m venv .venv`, upgraded pip, and ran `python -m pip install -e ".[dev]"` using the environment's interpreter. `python -m pytest` passed 3/3 cases; the same suite also passed from `build/no-tests/` using its absolute path. `ruff check .` and `ruff format --check .` passed. An isolated `python -I` import returned `aerotwin_experiments`, and `pip check` found no broken requirements. `cmake --preset dev-debug`, `cmake --build --preset dev-debug`, and `ctest --preset dev-debug` passed (3/3 C++ tests). The initial sandboxed pip attempt failed DNS resolution; retrying with approved network access succeeded. No packages were installed globally. Linux and other supported Python versions remain unverified; dependency ranges are not an exact lock.

### Step 0.7 — GitHub Actions Continuous Integration

**Status:** Complete

- [x] Configure CI for pull requests targeting `main` and pushes to `main`.
- [x] Configure C++ Debug and CTest on Ubuntu 24.04 and macOS 15 using the existing presets and GoogleTest FetchContent setup.
- [x] Configure editable installation, pytest, Ruff lint, and Ruff format checks on Ubuntu 24.04 with Python 3.11 and 3.13.
- [x] Document [local reproduction and failure triage](../../CONTRIBUTING.md#continuous-integration).
- [x] Record a successful hosted run covering all four matrix jobs.

**Local verification:** YAML parsed successfully. On Apple Silicon macOS, the Debug preset built and CTest passed 3/3 cases. A fresh Python 3.13.1 virtual environment passed editable installation, 3/3 pytest cases, Ruff lint, and Ruff format checks. `actionlint` was unavailable. These results cover the local environment.

**Hosted verification:** The GitHub Actions API reported success for C++ Debug on Ubuntu 24.04 and macOS 15 and for Python 3.11 and 3.13 on Ubuntu 24.04. The run corresponds to workflow commit `12ecc89`, merged as `2691531`. [CI run](https://github.com/msake592/aerotwin/actions/runs/41910316075), [PR #1](https://github.com/msake592/aerotwin/pull/1)

### Step 0.8 — Architecture Review & M0 Readiness

**Status:** Complete

- [x] Document the [architecture review and M1 readiness](../architecture/m0-review.md).
- [x] Record standalone core, deferred middleware, Python, frames, and simulation-time decisions.
- [x] Audit local build and test paths and record hosted CI evidence separately.
- [x] Document [Apple Silicon robotics feasibility](../architecture/robotics-integration-feasibility.md).

**Local audit:** Apple Silicon macOS with AppleClang 21.0.0 and CMake/CTest 4.4.4. Debug and Release configured and built, both examples ran, and CTest passed 3/3 cases for each preset. A fresh `BUILD_TESTING=OFF` build ran the example and produced zero registered tests and no FetchContent dependency directory. A fresh Python 3.13.1 virtual environment passed editable installation, pytest (3/3), Ruff lint, and Ruff formatting. Workflow YAML parsed and its triggers, job matrix, and test commands were reviewed. Representative generated paths matched `.gitignore`. `actionlint` was unavailable.

## M1 — Mathematical & Simulation Foundations

**Status:** Complete

**Objective:** Establish reliable mathematical contracts and a minimal standalone simulation basis.

**Main tasks:** Specify units, frames, timestamps, and state contracts; implement and test required mathematical primitives and a minimal deterministic single-drone simulation foundation.

**Deliverables:** Reviewed contracts, tested primitives, reproducible minimal simulation examples.

**Definition of done:** Numerical and frame tests pass; repeated seeded runs reproduce the documented baseline without ROS 2.

### Step M1.1 — Eigen Integration

**Status:** Merged in [PR #3](https://github.com/msake592/aerotwin/pull/3) at `0ad7fd5`. Local verification recorded below; hosted CI result not rechecked for M1.2.

Eigen 3.4.0 is fetched from its official release archive with SHA-256 verification.
The header-only `Eigen3::Eigen` target is a public dependency of `aerotwin_core`.
Eigen's standalone project is skipped; no Eigen tests, benchmarks, or examples are configured.
Five deterministic GoogleTest smoke tests cover construction/component access, addition,
dot product, matrix-vector multiplication, and identity behavior. C++20 and the existing
presets and CI jobs are preserved. No mathematical wrappers or later M1 steps are implemented.

**Local verification:** Apple Silicon macOS, AppleClang 21.0.0, CMake/CTest 4.4.4,
Unix Makefiles. `cmake --preset dev-debug`, `cmake --build --preset dev-debug`,
`ctest --preset dev-debug`, and the equivalent Release commands passed: 8/8 tests
in each configuration, including all three existing version regressions. Both
examples printed `AeroTwin version: 0.1.0-dev`. A fresh
`cmake -S . -B build/m1-no-tests -DBUILD_TESTING=OFF`, build, and example passed;
no GoogleTest dependency was populated and CTest registered zero tests.
After repairing the existing virtual environment with
`.venv/bin/python -m pip install -e '.[dev]'`, pytest passed 3/3 cases and Ruff
lint/format checks passed (Python 3.14.5, pytest 9.2.0, Ruff 0.17.3).
`pip check` and `git diff --check` passed. The initial sandboxed CMake download
stalled; configuration succeeded with approved network access. Existing CI jobs
are unchanged. Ubuntu, GitHub-hosted macOS, Python 3.11/3.13, and minimum CMake 3.20
were not tested for this change; hosted CI remains pending.

### Step M1.2 — Mathematical Types & Unit Contracts

**Status:** DONE. [PR #4](https://github.com/msake592/aerotwin/pull/4) merged into `main` at `1f2986b`; implementation commit `d0df059` is included.

- Eigen double-precision `Vector3` and Hamilton `Quaternion` aliases with explicit initialization requirements.
- Finite-vector and unit-quaternion validation without implicit repair.
- [Unit/frame/point contracts and transformation API draft](../architecture/math-contracts.md).
- [ADR 0006](../adr/0006-math-type-contracts.md) records aliases versus wrappers and invariant ownership.
- At M1.2 completion, quaternion operations (M1.3), frame transformations (M1.4), clocks, and simulation were deferred.

**Local verification:** Apple Silicon macOS, AppleClang 21.0.0, CMake/CTest 4.4.4.
`cmake --preset dev-debug`, `cmake --build --preset dev-debug`, and
`ctest --preset dev-debug` passed; equivalent Release commands passed. Both suites
passed 14/14 cases (all 8 existing tests plus 6 new contract tests). The existing
`build/m1-no-tests` configuration rebuilt with `BUILD_TESTING=OFF`; that example
and both preset examples printed `AeroTwin version: 0.1.0-dev` successfully.
Python 3.14.5 / pytest 9.2.0: `.venv/bin/python -m pytest` passed 3/3 after
repairing the missing editable package with `.venv/bin/python -m pip install -e '.[dev]'`.
An initial no-build-isolation repair could not import missing setuptools; the
standard isolated installation succeeded with approved network access.
Ruff 0.17.3: `.venv/bin/ruff check .` and `.venv/bin/ruff format --check .`
passed; `pip check` and `git diff --check` passed. No existing tests were removed.
Other platforms, Python 3.11/3.13, minimum CMake, and hosted M1.2 CI were not run locally.

**Hosted verification:** All four jobs passed in [CI run 42091533319](https://github.com/msake592/aerotwin/actions/runs/42091533319): C++ Debug/CTest on Ubuntu 24.04 and macOS 15, and Python tests plus Ruff checks on Ubuntu 24.04 with Python 3.11 and 3.13. This closed the M1.2 prerequisite for M1.3.

### Step M1.3 — Quaternion & Rotation Mathematics

**Status:** Merged in [PR #6](https://github.com/msake592/aerotwin/pull/6) at `65f5109`; all four jobs passed in [CI run 42124882965](https://github.com/msake592/aerotwin/actions/runs/42124882965).

Four checked operations normalize, compose (`after * before`), invert, and rotate
vectors using existing Eigen aliases and Hamilton conventions. `std::optional`
reports invalid input/nonfinite output. Rotation inputs must satisfy the existing
`1e-12` norm tolerance; accepted rounding drift is removed explicitly by policy.
See [rotation contracts](../architecture/quaternion-rotations.md) and
[ADR 0007](../adr/0007-quaternion-rotations.md). Full transformations, state models,
clocks, dynamics, and middleware remain deferred. PR #5 was verified merged with
all four CI jobs successful before implementation began.

**Local verification:** Apple Silicon macOS, AppleClang 21.0.0, CMake/CTest 4.4.4.
`cmake --preset dev-debug`, `cmake --build --preset dev-debug`, and
`ctest --preset dev-debug` passed; equivalent Release commands passed. Each suite
passed 35/35 cases, preserving all 14 existing cases and adding 21 rotation cases.
Coverage includes identity, WXYZ normalization, inverse, noncommuting composition,
Z-axis quarter turns, inverse round trips, norm preservation, sign equivalence,
invalid inputs, default tolerance boundaries, extreme finite normalization, and
vector output overflow. Eight additional numerical regressions cover identity and
known rotations with mixed-magnitude/subnormal vector components, overflow before
cancellation, mixed-magnitude quaternion normalization, accepted drift on a
non-identity rotation, and 10,000 repeated compositions. Three component-loss
regressions failed against the original unconditional scaling implementation,
then passed with unscaled finite rows and overflow-only row retries.
An additional 10,000-step composition test uses 0.1-radian increments and checks
finite/unit quaternions and the analytically accumulated rotation modulo 2*pi
with an absolute component tolerance of `1e-11`.
The existing `build/m1-no-tests` configuration rebuilt
with `BUILD_TESTING=OFF`; its example and both preset examples passed.
Python 3.14.5 / pytest 9.2.0: 3/3 passed after restoring the missing editable
package via `.venv/bin/python -m pip install -e '.[dev]'` with approved network
access. Ruff 0.17.3 lint and format checks, `pip check`, and `git diff --check`
passed. Changed documentation's relative file links were checked. Hosted M1.3 CI,
other platforms, Python 3.11/3.13, and minimum CMake 3.20 were not exercised.

### Step M1.4 — ENU/FLU Coordinate Transformations

**Status:** DONE. [PR #7](https://github.com/msake592/aerotwin/pull/7) merged into
`main` at `2979ff272c86f3f7533807c9534e6f150176e245`, including implementation
commit `817744ba8b0378ef8e3b3ba524712a66d3187fdb`.

- Four checked body/world point and vector functions reuse M1.3 rotations.
- Explicit active `q_world_from_body`, translation in world metres, and optional failures.
- [Coordinate contracts and usage](../architecture/coordinate-frames.md) document
  numerical limits and PX4 NED/FRD boundary mappings; no adapter is implemented.
- [ADR 0008](../adr/0008-coordinate-transformations.md) records the small API and
  rejection of nonfinite translation/subtraction intermediates.
- Fourteen new tests cover identity, all axes, quarter turns, inverse direction,
  translation order, round trips, invalid inputs, accepted quaternion drift,
  overflow, subnormals, mixed magnitudes, cancellation, and lost small offsets.
  The final integration regression verifies M1.3 internal-overflow recovery through
  all four M1.4 operations, using zero translation.

**Prerequisites:** Verified PR #6 merged at `65f5109` and all four CI jobs passed
via GitHub before changes. No open PRs (including documentation-only prerequisites)
were present. Switched to `main`, ran `git pull --ff-only origin main`, verified
clean state and merge ancestry, then created `feature/m1-coordinate-transformations`.

**Local verification:** Apple Silicon macOS, AppleClang 21.0.0, CMake/CTest 4.4.4.
`cmake --preset dev-debug`, `cmake --build --preset dev-debug`, and
`ctest --preset dev-debug` passed; equivalent Release commands passed. Both suites
passed 49/49 tests (35 preserved plus 14 new). Configured and built
`build/m1-no-tests` with `cmake -S . -B build/m1-no-tests -DBUILD_TESTING=OFF` and
`cmake --build build/m1-no-tests`; CTest listed zero tests. All three configurations'
examples printed `AeroTwin version: 0.1.0-dev` and exited successfully.
Python 3.14.5 / pytest 9.2.0: `.venv/bin/python -m pytest` passed 3/3.
Ruff 0.17.3: `.venv/bin/ruff check .` and `.venv/bin/ruff format --check .` passed;
`pip check`, changed documentation file-link checks, and `git diff --check` passed.
The editable installation was restored with `.venv/bin/python -m pip install -e '.[dev]'`
(network access was required after sandbox DNS failure). Import still failed because
macOS marked the editable `.pth` file hidden and Python skipped it; clearing that
flag on the generated `.venv` file fixed collection. No packaging changes were made.
Linux, Python 3.11/3.13, and minimum CMake 3.20 were not exercised locally.

**Hosted verification:** All four jobs passed in [CI run 42315065554](https://github.com/msake592/aerotwin/actions/runs/42315065554): C++ Debug/CTest on Ubuntu 24.04 and macOS 15, and pytest plus Ruff on Ubuntu 24.04 with Python 3.11 and 3.13.

### Step M1.5 — Simulation Clock

**Status:** DONE. Merged in [PR #9](https://github.com/msake592/aerotwin/pull/9) at `e34c264`.

The standalone core implements a fixed-step clock with signed 64-bit integer
nanoseconds, pause and resume, and reset to a new epoch. See the
[clock API and policies](../adr/0004-simulation-time.md).

**Local verification:** On Apple Silicon macOS, Debug and Release CTest passed
56/56 tests each, including seven clock tests. The tests-disabled build registered
zero tests. The version example ran in all three builds. Python 3.14.5 passed
pytest 3/3, Ruff lint and format checks, and `pip check`. Relative documentation
file links and `git diff --check` passed.

**Hosted verification:** All four jobs passed in [CI run 42385632519](https://github.com/msake592/aerotwin/actions/runs/42385632519).
The matrix covered C++ Debug on Ubuntu 24.04 and macOS 15 and Python 3.11 and 3.13 on Ubuntu 24.04.

### Step M1.6 — Vehicle State Model

**Status:** DONE. Merged in [PR #10](https://github.com/msake592/aerotwin/pull/10) at `f61c52f`.

The standalone core implements a validated ground-truth vehicle snapshot with
explicit ENU and FLU quantities and an M1.5 timestamp. See the
[vehicle state contract](../adr/0009-vehicle-state.md).

**Hosted verification:** All four jobs passed in [CI run 42405791742](https://github.com/msake592/aerotwin/actions/runs/42405791742).
The matrix covered C++ Debug on Ubuntu 24.04 and macOS 15 and Python 3.11 and 3.13 on Ubuntu 24.04.

### Step M1.7 — Kinematic Integrator

**Status:** DONE. Merged in [PR #11](https://github.com/msake592/aerotwin/pull/11) at `faeaad3`. All four hosted jobs passed in [CI run 42407157348](https://github.com/msake592/aerotwin/actions/runs/42407157348).

The standalone core propagates a ground-truth vehicle state to a caller-supplied
timestamp with constant world linear velocity and body angular velocity. See the
[integrator contract](../architecture/kinematic-integrator.md).

### Step M1.8 — Deterministic Scenario and Truth Telemetry

**Status:** DONE. Merged in [PR #12](https://github.com/msake592/aerotwin/pull/12) at `d22d53d`.

The runner records the initial snapshot and one validated snapshot per clock step.
See the [scenario and telemetry contract](../architecture/scenario-telemetry.md).
The phase gate passed 83/83 Debug and 83/83 Release tests before demo development.

### Step M1.9 — End-to-End Deterministic 3D Motion Demo

**Status:** DONE. Merged in [PR #12](https://github.com/msake592/aerotwin/pull/12) at `d22d53d`.

The headless reference scenario was verified against independent analytical
position and orientation results. See the [technical report](../reports/m1-deterministic-motion-report.md).

All four M1.8/M1.9 hosted CI jobs passed in [CI run 42411140548](https://github.com/msake592/aerotwin/actions/runs/42411140548).

## M2 — Sensor Simulation & State Estimation

**Status:** In Progress (M2.7 pending independent review)

**Objective:** Estimate single-drone state with explicit uncertainty.

**Main tasks:** Define sensor noise models; implement sensor simulation and a custom estimator such as an EKF; evaluate against separately routed truth.

**Deliverables:** Sensor models, estimator, fusion tests, documented noise assumptions and baseline evaluation.

**Definition of done:** Synthetic cases validate estimation and covariance behavior; seeded experiments report errors and failure cases without ground-truth leakage.

### Step M2.1 — Sensor Architecture and Measurement Contracts

**Status:** Merged in [PR #13](https://github.com/msake592/aerotwin/pull/13) at `c89eefe`.

Validated GPS and IMU measurements share acquisition metadata and use explicit
ENU/FLU units and covariance. See the
[measurement contract](../architecture/measurements.md) and
[ADR 0010](../adr/0010-measurement-contracts.md).

### Step M2.2 — Deterministic Noise Model

**Status:** Merged in [PR #14](https://github.com/msake592/aerotwin/pull/14) at `946ea28`. All four [hosted checks](https://github.com/msake592/aerotwin/actions/runs/42490435995) passed.

Caller-seeded Gaussian streams and validated covariance-based 3D sampling are
available to future sensor models. See the
[noise contract](../architecture/gaussian-noise.md).

### Step M2.3 — Deterministic GPS Sensor Simulation

**Status:** Merged in [PR #15](https://github.com/msake592/aerotwin/pull/15) at `0279c92`. All four [hosted checks](https://github.com/msake592/aerotwin/actions/runs/42492713457) passed.

The GPS model samples local ENU position from a supplied truth snapshot and
preserves acquisition time, sensor identity, and configured covariance. See the
[GPS simulation contract](../architecture/gps-simulation.md).

### Step M2.4 — Deterministic IMU Sensor Simulation

**Status:** Merged in [PR #16](https://github.com/msake592/aerotwin/pull/16) at `f2fc4c6`. All four [hosted checks](https://github.com/msake592/aerotwin/actions/runs/42522070318) passed.

The IMU generates FLU specific force and angular velocity from timestamp-matched
truth and ENU acceleration. See the [IMU contract](../architecture/imu-simulation.md).

### Step M2.5 — Deterministic Sensor Sampling Pipeline

**Status:** Merged in [PR #17](https://github.com/msake592/aerotwin/pull/17) at `67792c6`. All four [hosted checks](https://github.com/msake592/aerotwin/actions/runs/42594838522) passed.

The pipeline samples GPS and IMU at integer-nanosecond boundaries from exact-time
truth and returns ordered, owned measurement batches. See the
[sampling contract](../architecture/sensor-sampling.md).

### Step M2.6 — State Estimation Foundations

**Status:** Merged in [PR #18](https://github.com/msake592/aerotwin/pull/18) at `343a809`. All four [hosted checks](https://github.com/msake592/aerotwin/actions/runs/42669682825) passed.

The [estimator state contract](../architecture/state-estimation-foundations.md)
defines validated ENU position and velocity, their covariance, and initialization
at an exact simulation timestamp.

### Step M2.7 — Kalman Prediction

**Status:** Implemented locally, pending independent review.

[Kalman prediction](../architecture/kalman-prediction.md) advances an ENU estimate
with a constant-velocity model and propagates covariance with white-acceleration
process noise. Measurement correction remains planned.

## M3 — Gazebo, PX4 & ROS 2 Integration

**Status:** Not Started

**Objective:** Connect standalone algorithms to a validated robotics simulation stack.

**Main tasks:** Select compatible versions and environment; build adapters; connect Gazebo and PX4 SITL; validate frames, clocks, and flight-control boundaries.

**Deliverables:** Validated setup guide, integration packages, single-drone end-to-end scenario.

**Definition of done:** A documented scenario runs end to end; integration tests check data contracts and distinguish custom estimation from PX4 internal estimation.

## M4 — Uncertainty-Aware Autonomous Navigation

**Status:** Not Started

**Objective:** Use uncertainty explicitly in autonomous decisions.

**Main tasks:** Define navigation objectives and constraints; implement scoped planning/control interfaces; evaluate uncertainty-driven behavior and fallback cases.

**Deliverables:** Navigation components, scenarios, decision records and tests.

**Definition of done:** Reproducible scenarios demonstrate documented behavior under uncertainty and defined failure conditions.

## M5 — Validation & Robustness Benchmarking

**Status:** Not Started

**Objective:** Measure reliability under controlled disturbances and faults.

**Main tasks:** Define metrics and baselines; add fault and disturbance scenarios; automate repeatable experiment batches and analysis.

**Deliverables:** Versioned benchmark protocols, configurations, regression criteria and measured reports.

**Definition of done:** Reports include seeds, versions, assumptions and limitations; repeat runs satisfy stated reproducibility criteria and no results are fabricated.

## M6 — Explainable 3D Visualization

**Status:** Not Started

**Objective:** Make estimation uncertainty and autonomy behavior inspectable.

**Main tasks:** Visualize truth, estimates, uncertainty and decisions; support time-aligned inspection of recorded runs.

**Deliverables:** 3D visualization assets, replay/inspection workflow and explanatory examples.

**Definition of done:** A recorded scenario can be inspected with explicit frames and synchronized truth, estimates and decisions; visualization remains decoupled from algorithms.

## M7 — Multi-Agent Coordination

**Status:** Not Started

**Objective:** Extend validated single-agent foundations to coordinated agents.

**Main tasks:** Define agent identity and communication contracts; investigate coordination methods; evaluate multi-agent uncertainty and reliability.

**Deliverables:** Multi-agent scenarios, coordination interfaces and reproducible evaluation protocols.

**Definition of done:** Scoped multi-agent experiments meet documented coordination and reliability criteria and expose communication and scaling limitations.

