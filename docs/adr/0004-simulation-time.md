# ADR 0004: Use run-relative simulation timestamps

Status: Accepted design convention. M1.5 clock implementation merged in [PR #9](https://github.com/msake592/aerotwin/pull/9).

## Context

Simulation can pause, replay, or run faster than real time. Timing based on host execution makes deterministic tests and replay difficult.

## Decision

Represent elapsed simulation time as signed 64-bit integer nanoseconds from the simulation epoch, zero at run start. Simulated state and observations carry their event time and a run or epoch identifier. Simulation clock values are nondecreasing within an epoch. Concurrent events may share a timestamp, and delayed observations retain their capture time. `SimulationClock::reset()` advances the epoch, zeros elapsed time, and resumes the clock. Future simulation orchestration owns clearing buffered observations and other epoch-sensitive state. The clock has no observation buffers to clear.

## Rationale

Integer ticks avoid floating-point timestamp accumulation. Explicit epochs distinguish resets from late data and keep independent runs from being compared as one timeline.

## Consequences

Use wall time for external log correlation and a separate monotonic host clock for performance durations. Core time progression is driven by the simulation. M1 must specify step scheduling and test pause, repeatability, and reset behavior. Adapter synchronization and estimator handling of late data are deferred to the milestones that need them.

## Standalone clock API

Include `<aerotwin/simulation/SimulationClock.hpp>` and link `aerotwin_core`.
`SimulationClock::create(SimulationNanoseconds{10'000'000})` returns an optional
clock. The timestep must be strictly positive. Use an explicit nanosecond
duration. Other duration types require explicit conversion by the caller.

`now()` returns a value with an unsigned 64-bit `epoch` and an `elapsed` duration
with a signed 64-bit nanosecond count. Both begin at zero. Equality compares both
fields. Epochs are local to one clock's run history. Independent runs need an
external run identifier if their timestamps are exchanged.

- `timestep()` reads the fixed duration, and `isPaused()` reads the pause state.
- `step()` adds exactly one timestep and returns true on success. Elapsed time
  is nondecreasing within an epoch. The clock starts unpaused.
- `pause()` and `resume()` are idempotent and leave the timestamp unchanged.
- `reset()` increments the epoch, zeros elapsed time, and resumes the clock.
  Elapsed time can decrease across a reset. The timestep is retained.

A step while paused or beyond the maximum elapsed duration returns false.
Reset at the maximum epoch also returns false. Both checks happen before
mutation. The maximum representable value itself is valid. There is no
clamping or wraparound.

The same initial timestep and operation sequence produce identical timestamps.
Advancement depends only on explicit calls and integer arithmetic. A caller
owns and serializes access to its clock. Reset affects the clock alone. Future
simulation orchestration is responsible for resetting other state, discarding
buffered observations, and coordinating event times.
