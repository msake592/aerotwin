#include <aerotwin/sensors/Measurements.hpp>

#include "Covariance.hpp"

#include <algorithm>

namespace aerotwin {

std::optional<MeasurementAcquisition> MeasurementAcquisition::create(
    std::string_view sensor_id, SimulationTimestamp timestamp) noexcept {
    constexpr std::string_view allowed =
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-";
    if (sensor_id.empty() || sensor_id.size() > 64 ||
        sensor_id.find_first_not_of(allowed) != std::string_view::npos ||
        timestamp.elapsed < SimulationNanoseconds::zero()) {
        return std::nullopt;
    }
    return MeasurementAcquisition(sensor_id, timestamp);
}

MeasurementAcquisition::MeasurementAcquisition(std::string_view sensor_id,
                                               SimulationTimestamp timestamp) noexcept
    : size_(sensor_id.size()), timestamp_(timestamp) {
    std::copy(sensor_id.begin(), sensor_id.end(), sensor_id_.begin());
}

std::optional<GpsMeasurement> GpsMeasurement::create(
    MeasurementAcquisition acquisition, const Vector3& position_world_m,
    const Eigen::Matrix3d& position_covariance_world_m2) {
    if (!isFinite(position_world_m)) {
        return std::nullopt;
    }
    const auto covariance = detail::validatedCovariance(position_covariance_world_m2);
    if (!covariance) {
        return std::nullopt;
    }
    return GpsMeasurement(acquisition, position_world_m, *covariance);
}

GpsMeasurement::GpsMeasurement(MeasurementAcquisition acquisition, const Vector3& position_world_m,
                               const Eigen::Matrix3d& covariance) noexcept
    : acquisition_(acquisition), position_world_m_(position_world_m),
      position_covariance_world_m2_(covariance) {}

std::optional<ImuMeasurement> ImuMeasurement::create(
    MeasurementAcquisition acquisition, const Vector3& specific_force_body_mps2,
    const Vector3& angular_velocity_body_radps,
    const Eigen::Matrix3d& specific_force_covariance_body_m2ps4,
    const Eigen::Matrix3d& angular_velocity_covariance_body_rad2ps2) {
    if (!isFinite(specific_force_body_mps2) || !isFinite(angular_velocity_body_radps)) {
        return std::nullopt;
    }
    const auto force_covariance = detail::validatedCovariance(specific_force_covariance_body_m2ps4);
    const auto rate_covariance = detail::validatedCovariance(angular_velocity_covariance_body_rad2ps2);
    if (!force_covariance || !rate_covariance) {
        return std::nullopt;
    }
    return ImuMeasurement(acquisition, specific_force_body_mps2, angular_velocity_body_radps,
                          *force_covariance, *rate_covariance);
}

ImuMeasurement::ImuMeasurement(MeasurementAcquisition acquisition,
                               const Vector3& specific_force_body_mps2,
                               const Vector3& angular_velocity_body_radps,
                               const Eigen::Matrix3d& specific_force_covariance,
                               const Eigen::Matrix3d& angular_velocity_covariance) noexcept
    : acquisition_(acquisition), specific_force_body_mps2_(specific_force_body_mps2),
      angular_velocity_body_radps_(angular_velocity_body_radps),
      specific_force_covariance_body_m2ps4_(specific_force_covariance),
      angular_velocity_covariance_body_rad2ps2_(angular_velocity_covariance) {}

} // namespace aerotwin
