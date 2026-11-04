# Contributing to AeroTwin

Read [AGENTS.md](AGENTS.md), the [architecture overview](docs/architecture/overview.md), and the [roadmap](docs/roadmap/ROADMAP.md) before changing the project.

## Workflow

1. Select a scoped task in the current milestone.
2. Create a feature branch, such as `codex/document-foundation` for agent work or `feat/short-description` for other contributions.
3. Make focused changes and preserve unrelated user work.
4. Verify the change and open a pull request using the repository template.
5. Describe behavior, milestone, validation evidence, and limitations for review.

## Testing expectations

New functionality requires relevant tests, including numerical and failure cases where applicable. Never delete failing tests just to pass CI. Report exact commands and results, including tests that could not run. Do not invent benchmarks.

The standalone C++20 library and example can be built in Debug and Release using the commands in [README.md](README.md#getting-started). Verify both builds and the example output for build-system changes. GoogleTest provides unit-test assertions; CTest discovers and runs the cases. Add unit tests under `core/tests/unit/`, register new source files in `core/tests/CMakeLists.txt`, rebuild, and run `ctest --preset dev-debug` and `ctest --preset dev-release`. The test presets fail if no tests are found and show failures. Keep GoogleTest test-only; never include it in core public headers. It is pinned to v1.18.0 and downloaded through FetchContent into the build tree, requiring network access for a fresh testing-enabled configuration. See the [README testing instructions](README.md#unit-tests) for equivalent CTest commands and multi-configuration generators.

For build-system changes, also verify `cmake -S . -B build/no-tests -DBUILD_TESTING=OFF`, `cmake --build build/no-tests`, and the example in that directory. This configuration must not fetch or configure GoogleTest. Keep integration tests deferred until they exercise meaningful integration behavior. Documentation changes should verify file existence, relative links, and consistency with implemented status. CI configuration remains deferred. Platform verification and results are recorded in the [roadmap](docs/roadmap/ROADMAP.md); report limitations for environments you cannot exercise.

## Python development

Use Python 3.11+ and a project-local `.venv`; follow the [README setup](README.md#python-development-environment). Activate the environment and install `python -m pip install -e ".[dev]"`. Do not install packages globally or modify `PYTHONPATH` to make tests pass.

Keep Python sources in `experiments/python/src/aerotwin_experiments/` and tests in `tests/python/`. setuptools provides the pure-Python build backend; pytest and Ruff are development-only dependencies. Keep runtime dependencies minimal and justify additions. The editable-installation test intentionally requires the documented editable development installation.

Before submitting Python changes, run:

```sh
python -m pytest
ruff check .
ruff format --check .
cmake --preset dev-debug
cmake --build --preset dev-debug
ctest --preset dev-debug
```

pytest uses importlib mode and the installed package; test paths and Ruff rules are configured in `pyproject.toml`. Ruff checks basic errors and import ordering with a 100-character formatter setting and Python 3.11 target. Python is reserved for experiment orchestration and analysis; numerical/autonomy core implementations remain in C++. Do not introduce bindings, robotics frameworks, scientific libraries, or future experiment functionality without a scoped task. Record the interpreter, dependency versions, platform, and checks actually run; Linux compatibility is intended but not yet verified.

## Documentation and design

Update README and roadmap status when behavior changes. Record major architectural decisions as numbered ADRs. Document frames, units, timestamps, sensor assumptions, and non-obvious choices. Keep the core independent of ROS 2 and avoid dependencies outside the requested milestone.

## Commit conventions

Use concise imperative subjects with a type: `docs: describe module boundaries`, `feat: add requested capability`, `fix: correct behavior`, `test: cover failure case`, or `chore: maintain tooling`. Explain the reason for non-obvious changes in the body. Keep commits focused; do not claim releases or successful checks without evidence.
