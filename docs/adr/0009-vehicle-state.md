# ADR 0009 — Validated simulator vehicle state

Status: Accepted. Merged in [PR #10](https://github.com/msake592/aerotwin/pull/10).

`aerotwin::VehicleState` belongs to the simulator truth layer. Future sensor
models may derive measurements from this snapshot. Estimators must receive
measurements through separate contracts and produce a separate estimated state.
Validation can compare estimates against separately routed truth.

## State contract

The public header is `<aerotwin/simulation/VehicleState.hpp>`. Link
`aerotwin_core` and use the existing [frame conventions](../architecture/coordinate-frames.md).

| Accessor | Type | Meaning |
| --- | --- | --- |
| `positionWorldM()` | `Vector3` | Body origin in world ENU, metres |
| `orientationWorldFromBody()` | `Quaternion` | Active Hamilton rotation mapping body FLU vectors into world ENU, with WXYZ constructor order |
| `linearVelocityWorldMps()` | `Vector3` | Velocity of the body origin in world ENU, metres per second |
| `angularVelocityBodyRadps()` | `Vector3` | Angular velocity expressed in body FLU, radians per second |
| `timestamp()` | `SimulationTimestamp` | Unsigned 64-bit clock epoch and signed 64-bit elapsed nanoseconds within that epoch |

A snapshot preserves the supplied timestamp. The caller owns clock provenance
and supplies the event time, typically from `SimulationClock::now()`. Clock
reset advances the epoch and zeros elapsed time. Epochs are local to a clock
history. See the [simulation time contract](0004-simulation-time.md).

## Construction and ownership

`VehicleState::create` returns `std::nullopt` for non-finite vector components,
invalid quaternion components, zero or non-unit quaternion norm beyond the
existing absolute tolerance of `1e-12`, or negative elapsed time. Every unsigned
epoch and every nonnegative elapsed value is representable. Position and velocity
have no physical magnitude bounds. The factory preserves accepted quaternion
coefficients, including allowed norm drift, and leaves inputs untouched.

The snapshot owns its values and returns copies. It has no default constructor
or field setters. Callers may replace the whole snapshot with another validated
value. Frame, unit, and event-time consistency remain caller responsibilities.
Using the existing aliases keeps interoperability with Eigen and the rotation
and transformation APIs. An optional result conveys failure without identifying
the invalid field.

The [M1.7 kinematic integrator](../architecture/kinematic-integrator.md) consumes
a snapshot and constructs a new validated snapshot at the supplied timestamp.
Clock advancement and propagation live outside the state model.

```cpp
#include <aerotwin/simulation/VehicleState.hpp>

// At run start, epoch and elapsed nanoseconds are both zero.
const auto state = aerotwin::VehicleState::create(
    aerotwin::Vector3::Zero(), aerotwin::Quaternion::Identity(),
    aerotwin::Vector3::Zero(), aerotwin::Vector3::Zero(),
    aerotwin::SimulationTimestamp{});
```
