# ADR 0002: Use Python for experiments and analysis

Status: Accepted (package scaffold implemented; workflows planned)

## Context

Experiment configuration, run orchestration, and reporting evolve at a different pace from numerical algorithms.

## Decision

Keep experiment tooling in the Python package under `experiments/`. Exchange explicit configuration and versioned results across the boundary when an experiment first needs them. Keep sensor, estimation, and autonomy algorithms in the standalone C++ core.

## Rationale

Python supports iterative analysis while preserving a standalone, testable C++ build and a separate packaging lifecycle.

## Consequences

Run records will need scenario parameters, seeds, tool versions, and schema identifiers to support replay and comparison. Python currently provides package identity and packaging checks. Bindings and transport can be evaluated when a concrete workflow requires them.
