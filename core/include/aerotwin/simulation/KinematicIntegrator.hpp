#pragma once

#include <aerotwin/simulation/VehicleState.hpp>

#include <optional>

namespace aerotwin {

// Propagates simulator truth with constant world linear and body angular velocity.
// The target must be in the same epoch and at or after the initial timestamp.
// An equal timestamp returns an unchanged snapshot. Returns nullopt on invalid
// time progression or a non-finite or invalid computed state.
[[nodiscard]] std::optional<VehicleState> integrateConstantVelocity(
    const VehicleState& initial_state, SimulationTimestamp target_timestamp);

} // namespace aerotwin
