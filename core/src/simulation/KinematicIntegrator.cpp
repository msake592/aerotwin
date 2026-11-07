#include <aerotwin/simulation/KinematicIntegrator.hpp>

#include <aerotwin/math/Rotations.hpp>

#include <cmath>

namespace aerotwin {

std::optional<VehicleState> integrateConstantVelocity(
    const VehicleState& initial_state, SimulationTimestamp target_timestamp) {
    const auto initial_timestamp = initial_state.timestamp();
    if (target_timestamp.epoch != initial_timestamp.epoch ||
        target_timestamp.elapsed < initial_timestamp.elapsed) {
        return std::nullopt;
    }
    if (target_timestamp == initial_timestamp) {
        return initial_state;
    }

    // Both counts are nonnegative and ordered, so subtraction fits in int64_t.
    const auto elapsed = target_timestamp.elapsed - initial_timestamp.elapsed;
    const double seconds = std::chrono::duration<double>(elapsed).count();

    const Vector3 position = initial_state.positionWorldM();
    const Vector3 linear_velocity = initial_state.linearVelocityWorldMps();
    const Vector3 next_position(std::fma(linear_velocity.x(), seconds, position.x()),
                                std::fma(linear_velocity.y(), seconds, position.y()),
                                std::fma(linear_velocity.z(), seconds, position.z()));

    const Vector3 angular_velocity = initial_state.angularVelocityBodyRadps();
    Quaternion orientation = initial_state.orientationWorldFromBody();
    const Vector3 half_rotation = angular_velocity * (0.5 * seconds);
    const double half_angle = std::hypot(half_rotation.x(), half_rotation.y(), half_rotation.z());
    if (!std::isfinite(half_angle)) {
        return std::nullopt;
    }
    if (half_angle > 0.0) {
        // sin(x)/x is indistinguishable from one at this scale in double precision.
        // For larger angles, divide first to keep the vector part bounded.
        Vector3 imaginary = half_rotation;
        if (half_angle >= 1e-8) {
            imaginary = (half_rotation / half_angle) * std::sin(half_angle);
        }
        const Quaternion body_increment(std::cos(half_angle), imaginary.x(), imaginary.y(),
                                        imaginary.z());
        const auto composed = composeRotations(orientation, body_increment);
        if (!composed) {
            return std::nullopt;
        }
        orientation = *composed;
    }

    return VehicleState::create(next_position, orientation, linear_velocity, angular_velocity,
                                target_timestamp);
}

} // namespace aerotwin
