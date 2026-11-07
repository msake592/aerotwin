#include <aerotwin/simulation/VehicleState.hpp>

namespace aerotwin {

std::optional<VehicleState> VehicleState::create(
    const Vector3& position_world_m, const Quaternion& orientation_world_from_body,
    const Vector3& linear_velocity_world_mps, const Vector3& angular_velocity_body_radps,
    SimulationTimestamp timestamp) noexcept {
    if (!isFinite(position_world_m) || !isUnitQuaternion(orientation_world_from_body) ||
        !isFinite(linear_velocity_world_mps) || !isFinite(angular_velocity_body_radps) ||
        timestamp.elapsed < SimulationNanoseconds::zero()) {
        return std::nullopt;
    }

    return VehicleState(position_world_m, orientation_world_from_body, linear_velocity_world_mps,
                        angular_velocity_body_radps, timestamp);
}

VehicleState::VehicleState(const Vector3& position_world_m,
                           const Quaternion& orientation_world_from_body,
                           const Vector3& linear_velocity_world_mps,
                           const Vector3& angular_velocity_body_radps,
                           SimulationTimestamp timestamp) noexcept
    : position_world_m_(position_world_m), orientation_world_from_body_(orientation_world_from_body),
      linear_velocity_world_mps_(linear_velocity_world_mps),
      angular_velocity_body_radps_(angular_velocity_body_radps), timestamp_(timestamp) {}

} // namespace aerotwin
