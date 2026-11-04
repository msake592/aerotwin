# ADR 0004: Use run-relative simulation timestamps

Status: Accepted as a design convention (clock implementation and tests pending)

## Context

Simulation can pause, replay, or run faster than real time. Timing based on host execution makes deterministic tests and replay difficult.

## Decision

Represent elapsed simulation time as signed 64-bit integer nanoseconds from the simulation epoch, zero at run start. Simulated state and observations carry their event time and a run or epoch identifier. Simulation clock values are nondecreasing within an epoch. Concurrent events may share a timestamp, and delayed observations retain their capture time. A reset starts a new epoch at zero and clears time-dependent state and buffered observations.

## Rationale

Integer ticks avoid floating-point timestamp accumulation. Explicit epochs distinguish resets from late data and keep independent runs from being compared as one timeline.

## Consequences

Use wall time for external log correlation and a separate monotonic host clock for performance durations. Core time progression is driven by the simulation. M1 must specify step scheduling and test pause, repeatability, and reset behavior. Adapter synchronization and estimator handling of late data are deferred to the milestones that need them.
