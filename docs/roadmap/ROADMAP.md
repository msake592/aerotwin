# Development roadmap

Steps 0.1–0.5 are implemented: repository foundation documentation, a minimal standalone C++20 build with a version-reporting example, and optional GoogleTest/CTest unit testing. Simulation, estimation, autonomy, and CI remain planned. Milestone completion requires evidence against its definition of done.

## M0 — Project Foundation

**Status:** In Progress

**Objective:** Establish a documented and reviewable engineering foundation.

**Main tasks:** Create the repository structure and initial documentation; establish engineering guidance and an ADR; establish the standalone C++20 build; establish optional unit testing; scope later CI tasks.

**Deliverables:** Repository skeleton, README, contribution guidance, architecture documents, roadmap, license, formatting and ignore rules, CMake targets and presets, a linked version-reporting example, and discovered GoogleTest unit tests.

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

### Remaining M0 work
- [ ] Configure and validate CI in a subsequent step.
- [ ] Complete any remaining environment/setup validation before closing M0.

## M1 — Mathematical & Simulation Foundations

**Status:** Not Started

**Objective:** Establish reliable mathematical contracts and a minimal standalone simulation basis.

**Main tasks:** Specify units, frames, timestamps, and state contracts; implement and test required mathematical primitives and a minimal deterministic single-drone simulation foundation.

**Deliverables:** Reviewed contracts, tested primitives, reproducible minimal simulation examples.

**Definition of done:** Numerical and frame tests pass; repeated seeded runs reproduce the documented baseline without ROS 2.

## M2 — Sensor Simulation & State Estimation

**Status:** Not Started

**Objective:** Estimate single-drone state with explicit uncertainty.

**Main tasks:** Define sensor noise models; implement sensor simulation and a custom estimator such as an EKF; evaluate against separately routed truth.

**Deliverables:** Sensor models, estimator, fusion tests, documented noise assumptions and baseline evaluation.

**Definition of done:** Synthetic cases validate estimation and covariance behavior; seeded experiments report errors and failure cases without ground-truth leakage.

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

