#pragma once

#include <aerotwin/math/Types.hpp>
#include <aerotwin/simulation/SimulationClock.hpp>

#include <optional>

namespace aerotwin {

// Validated simulator ground truth. Estimators consume separate measurement contracts.
class VehicleState {
public:
    // Rejects non-finite vectors, invalid orientations, and negative elapsed time.
    // Accepted quaternion coefficients are preserved without normalization.
    [[nodiscard]] static std::optional<VehicleState> create(
        const Vector3& position_world_m, const Quaternion& orientation_world_from_body,
        const Vector3& linear_velocity_world_mps, const Vector3& angular_velocity_body_radps,
        SimulationTimestamp timestamp) noexcept;

    // Body origin in world ENU, metres.
    [[nodiscard]] Vector3 positionWorldM() const noexcept { return position_world_m_; }
    // Active Hamilton rotation from body FLU to world ENU.
    [[nodiscard]] Quaternion orientationWorldFromBody() const noexcept {
        return orientation_world_from_body_;
    }
    // World ENU, metres per second.
    [[nodiscard]] Vector3 linearVelocityWorldMps() const noexcept {
        return linear_velocity_world_mps_;
    }
    // Body FLU, radians per second.
    [[nodiscard]] Vector3 angularVelocityBodyRadps() const noexcept {
        return angular_velocity_body_radps_;
    }
    [[nodiscard]] SimulationTimestamp timestamp() const noexcept { return timestamp_; }

private:
    VehicleState(const Vector3& position_world_m, const Quaternion& orientation_world_from_body,
                 const Vector3& linear_velocity_world_mps,
                 const Vector3& angular_velocity_body_radps, SimulationTimestamp timestamp) noexcept;

    Vector3 position_world_m_;
    Quaternion orientation_world_from_body_;
    Vector3 linear_velocity_world_mps_;
    Vector3 angular_velocity_body_radps_;
    SimulationTimestamp timestamp_;
};

} // namespace aerotwin
