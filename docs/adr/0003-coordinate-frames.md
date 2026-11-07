# ADR 0003: Declare coordinate frames and attitude conventions

Status: Accepted as a design convention (M1.4 body/world transformations merged in PR #7)

## Context

Core geometry and PX4 use different world and body axes. Implicit conversion can yield plausible results with incorrect signs or orientations.

## Decision

Use the [frame and units contract](../architecture/coordinate-frames.md) for core-facing interfaces. Convert external representations explicitly at adapters.

## Rationale

ENU and FLU preserve the existing design direction. A single quaternion order and direction give tests and data interchange a common interpretation.

## Consequences

M1 must test basis vectors, rotation direction, normalization, and round trips. Adapters must declare external alignment, origin, and covariance ordering and validate their conversions before integration.
