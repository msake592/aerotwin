# AeroTwin

Autonomous Multi-Agent Digital Twin & Sensor Fusion Platform.

AeroTwin is a planned modular 3D simulation and validation platform for autonomous aerial systems. Development starts with one simulated drone and will later explore multi-agent coordination.

## Motivation

Autonomous flight depends on imperfect measurements, uncertain state estimates, and changing environments. Reproducible experiments should make the relationship between uncertainty, autonomy decisions, and system reliability measurable and explainable.

## Core research question

How can uncertainty-aware sensor fusion and autonomy improve the reliability of aerial systems under sensor noise, failures, and environmental disturbances, while producing reproducible and explainable evidence?

## Planned features

- Configurable simulation scenarios and explicit ground-truth references.
- Sensor models and independent state estimation, including an EKF.
- Uncertainty-aware navigation and robustness evaluation.
- Reproducible experiment orchestration, metrics, and reports.
- Explainable 3D views of truth, estimates, uncertainty, and decisions.

Multi-agent coordination is a future research direction. None of these runtime capabilities is implemented yet.

## High-level architecture

Gazebo is intended to supply simulation and ground truth; PX4 SITL will provide flight control. ROS 2 adapters will connect these systems to a standalone C++ core. The custom estimator will remain independent of PX4's internal estimator. Python tools will orchestrate experiments and analyze results. See the [architecture overview](docs/architecture/overview.md) and [module boundaries](docs/architecture/module-boundaries.md).

## Planned technology stack

| Area | Planned tools |
| --- | --- |
| Core | C++20 and CMake (implemented); Eigen (planned) |
| Integration | ROS 2, Gazebo, PX4 SITL |
| Experiments and analysis | Python |
| Tests | GoogleTest, CTest, and pytest (implemented) |
| Automation | GitHub Actions |

CMake 3.20+ and a C++20 compiler are required for the current build. Robotics dependency versions and installation are deferred to later foundation work. The primary machine is an Apple Silicon MacBook without an NVIDIA CUDA GPU; core development must remain usable without NVIDIA-specific hardware.

## Current development status

M0 — Project Foundation is **In Progress**. Steps 0.1–0.3 establish directory structure, engineering guidance, and project documentation. Step 0.4 adds a standalone C++20 static library (`aerotwin_core`), a version-reporting example, and Debug/Release CMake presets. Both builds and example executions have been verified on Apple Silicon macOS. Step 0.5 adds optional GoogleTest unit tests and CTest discovery. Step 0.6 adds an installable Python experiment package, pytest, and Ruff development tooling. Simulation, EKF, navigation, ROS 2 packages, and CI remain unimplemented.

## Development roadmap

| Milestone | Focus | Status |
| --- | --- | --- |
| M0 | Project Foundation | In Progress |
| M1 | Mathematical & Simulation Foundations | Not Started |
| M2 | Sensor Simulation & State Estimation | Not Started |
| M3 | Gazebo, PX4 & ROS 2 Integration | Not Started |
| M4 | Uncertainty-Aware Autonomous Navigation | Not Started |
| M5 | Validation & Robustness Benchmarking | Not Started |
| M6 | Explainable 3D Visualization | Not Started |
| M7 | Multi-Agent Coordination | Not Started |

See the [detailed roadmap](docs/roadmap/ROADMAP.md) and [milestone dependencies](docs/roadmap/MILESTONES.md).

## Documentation structure

- [Architecture](docs/architecture/overview.md): intended responsibilities, data flow, and frame conventions.
- [ADR 0001](docs/adr/0001-core-independent-of-ros.md): standalone core decision.
- [Roadmap](docs/roadmap/ROADMAP.md): tasks, deliverables, and completion criteria.
- `docs/design/`: reserved for detailed designs.
- `docs/setup/`: reserved for validated setup instructions.
- [Contributing](CONTRIBUTING.md) and [agent guidance](AGENTS.md): engineering workflow.

`core/` holds the minimal version-reporting library and space for future algorithms and tests; `simulation/` holds future simulation assets; `ros2_ws/` holds future adapters. `experiments/`, `visualization/`, `tools/`, `examples/`, `tests/`, and `data/` reserve their respective responsibilities. Generated results belong in ignored `outputs/`.

## Getting started

1. Open the existing repository root.
2. Read [CONTRIBUTING.md](CONTRIBUTING.md), [AGENTS.md](AGENTS.md), and the [roadmap](docs/roadmap/ROADMAP.md).
3. Choose the next explicitly scoped foundation task before adding code or dependencies.

### Required tools

- CMake 3.20 or newer.
- A C++20-capable compiler (AppleClang, Clang, GCC, or MSVC).
- Make for the supplied presets. On macOS, the Xcode Command Line Tools provide the compiler, SDK, and Make.

The standalone core requires no third-party library. Testing is enabled by default and downloads pinned GoogleTest v1.18.0 using CMake FetchContent; network access is needed on first configuration of each build directory. A system-wide GoogleTest installation is unnecessary. Downloads and compiled dependencies stay under ignored `build/`. No ROS 2, Gazebo, PX4, CUDA, or Eigen is required. Ninja is optional and is not required by the presets.

### Configure, build, and run

Run from the repository root:

```sh
cmake --preset dev-debug
cmake --build --preset dev-debug
./build/dev-debug/examples/basic_core/aerotwin_basic_core

cmake --preset dev-release
cmake --build --preset dev-release
./build/dev-release/examples/basic_core/aerotwin_basic_core
```

Both examples should print `AeroTwin version: 0.1.0-dev` and exit successfully. Generated files stay under ignored `build/`. The targets use C++20 without compiler extensions and enable compiler warnings without treating them as errors.

The presets use `Unix Makefiles`, suitable for macOS and Linux with Make installed. To use another generator, configure a separate build directory with CMake's platform default (or supply `-G` for an installed generator):

```sh
cmake -S . -B build/native-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/native-debug --config Debug
```

`CMAKE_BUILD_TYPE` selects the configuration for single-configuration generators. For multi-configuration generators such as Visual Studio, `--config Debug` selects it at build time; run `build/native-debug/examples/basic_core/Debug/aerotwin_basic_core.exe`. Use a separate directory and `Release` in place of `Debug` for an equivalent Release build. Non-macOS platforms have not yet been verified.

There are no installation or simulation launch commands yet.

### Unit tests

GoogleTest supplies C++ assertions and the test executable. CTest is CMake's test runner; `gtest_discover_tests` automatically registers each GoogleTest case with CTest. GoogleTest is linked privately by the test executable and is absent from the core's public interface.

After configuring and building as above, run:

```sh
ctest --preset dev-debug
ctest --preset dev-release
```

Equivalent commands using the existing build paths are:

```sh
ctest --test-dir build/dev-debug --output-on-failure --no-tests=error
ctest --test-dir build/dev-release --output-on-failure --no-tests=error
```

For multi-configuration generators, also specify `-C Debug` or `-C Release` when invoking CTest. Rebuild before running tests after source changes; CTest does not build the executable.

To disable testing and avoid downloading or configuring GoogleTest, use a separate build directory:

```sh
cmake -S . -B build/no-tests -DBUILD_TESTING=OFF
cmake --build build/no-tests
./build/no-tests/examples/basic_core/aerotwin_basic_core
```

The development presets explicitly enable `BUILD_TESTING`; the no-tests build leaves the library and example available. On multi-configuration generators, build with `--config Debug` and use the configuration-specific executable path described above.

### Python development environment

The Python experiment layer supports Python 3.11+ on macOS and Linux. It is intended for future experiment orchestration, telemetry analysis, metrics, reporting, and reproducible research. Currently it exposes only `package_name()` to verify installation and imports. Performance-critical mathematics, sensor models, estimation, and autonomy belong in the standalone C++ core. Python does not replace that core; no C++ bindings or robotics dependencies are introduced.

A root `pyproject.toml` uses setuptools with sources in `experiments/python/src/`, keeping Python packaging within the experiment layer and preserving the existing experiment directories. Runtime dependencies are empty. The `dev` extra contains pytest and Ruff; setuptools is an isolated build dependency. Package metadata versioning does not call or duplicate the C++ version function.

From the repository root, select a Python 3.11+ interpreter (`python3 --version`) and run:

```sh
python3 -m venv .venv
source .venv/bin/activate
python --version
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -c "from aerotwin_experiments import package_name; print(package_name())"
python -m pytest
ruff check .
ruff format --check .
```

The import command should print `aerotwin_experiments`. Editable installation makes source edits available immediately. Tests check imports, identity, and the editable installation's metadata/source location without `PYTHONPATH` changes. To check imports independently of the working directory, the equivalent isolated interpreter check is:

```sh
python -I -c "from aerotwin_experiments import package_name; print(package_name())"
```

pytest and Ruff configuration lives in `pyproject.toml`: tests are under `tests/python/`; Ruff uses Python 3.11 syntax, a 100-character line length, basic error checks, import sorting, and its formatter. Ruff excludes generated build and output trees. Use `ruff format .` to apply formatting. No Python commands are coupled to the CMake build.

`.venv/`, caches, generated package metadata, and build artifacts are ignored. Inspect an existing environment before reusing it; do not delete it automatically. Dependencies install only in the virtual environment, with network access needed for initial setup. Development dependency ranges allow compatible updates; this step does not provide an exact dependency lock or claim bit-for-bit environment reproduction. Linux and Python versions other than the locally verified interpreter remain untested.

## Testing status

Python Step 0.6 verification passed on Apple Silicon macOS using Python 3.14.5, pytest 9.2.0, and Ruff 0.17.3: all three Python tests passed from both the repository root and another directory inside it; linting, formatting, isolated import, and dependency checks passed. The C++ Debug build and its three tests also passed unchanged. Initial sandboxed pip DNS restrictions were resolved with approved network access. Python 3.11–3.13 and Linux have not been exercised.

The initial C++ unit suite checks that `aerotwin::version()` returns a non-null pointer, a non-empty string, and the expected `0.1.0-dev` version. Verified on Apple Silicon macOS with AppleClang 21.0.0 and CMake/CTest 4.4.4: Debug and Release each passed all three tests, and the testing-disabled build and example passed without fetching GoogleTest. Other platforms and CMake 3.20 remain untested. The initial sandboxed fetch failed DNS resolution; approved network access resolved it. Full verification results are recorded in the [roadmap](docs/roadmap/ROADMAP.md). Integration tests remain deferred until meaningful integration behavior exists; CI is a subsequent step. Future functionality must include relevant tests; no performance or reliability results are claimed.

## License

[MIT](LICENSE), copyright 2026 AeroTwin Contributors.
