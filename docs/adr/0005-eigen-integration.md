# ADR 0005: Use pinned Eigen headers for core linear algebra

Status: Proposed (M1.1 implementation pending review/merge and hosted CI)

## Context

The standalone C++20 core needs linear algebra before mathematical contracts and
simulation are implemented. The project already uses CMake FetchContent for GoogleTest.

## Decision

Use Eigen 3.4.0 from the official GitLab release archive, pinned with SHA-256
`8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72`.
FetchContent downloads and extracts the archive. A deliberately absent
`SOURCE_SUBDIR` skips Eigen's standalone CMake project. AeroTwin supplies the
header-only imported interface target `Eigen3::Eigen`, linked publicly by
`aerotwin_core`. Include paths stay target-scoped.

## Rationale

Eigen provides established vector and matrix operations without middleware or
hardware dependencies. Consuming only headers avoids configuring Eigen's own
tests, benchmarks, examples, and standalone compiler settings, including its
shared `BUILD_TESTING` option. A system package would make the selected version
depend on the developer or runner environment; vendoring would add library sources
to this repository.

## Consequences

Fresh build directories require network access even when tests are disabled.
Downloaded files remain in the ignored build tree, and archive hash verification
rejects changed content. No compiled Eigen library or system installation is
required. Existing CMake/compiler requirements remain unchanged. Five smoke tests
exercise the dependency through the core's transitive interface. Mathematical type
contracts, frames, quaternions, and simulation remain separate future steps.
