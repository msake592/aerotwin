#pragma once

#include <aerotwin/math/Types.hpp>
#include <aerotwin/simulation/SimulationClock.hpp>

#include <array>
#include <optional>
#include <string_view>

namespace aerotwin {

class MeasurementAcquisition {
public:
    // Sensor IDs contain 1–64 ASCII letters, digits, periods, underscores, or hyphens.
    [[nodiscard]] static std::optional<MeasurementAcquisition> create(
        std::string_view sensor_id, SimulationTimestamp timestamp) noexcept;

    [[nodiscard]] std::string_view sensorId() const noexcept { return {sensor_id_.data(), size_}; }
    [[nodiscard]] SimulationTimestamp timestamp() const noexcept { return timestamp_; }

private:
    MeasurementAcquisition(std::string_view sensor_id, SimulationTimestamp timestamp) noexcept;

    std::array<char, 64> sensor_id_{};
    std::size_t size_;
    SimulationTimestamp timestamp_;
};

class GpsMeasurement {
public:
    [[nodiscard]] static std::optional<GpsMeasurement> create(
        MeasurementAcquisition acquisition, const Vector3& position_world_m,
        const Eigen::Matrix3d& position_covariance_world_m2);

    [[nodiscard]] const MeasurementAcquisition& acquisition() const noexcept { return acquisition_; }
    [[nodiscard]] Vector3 positionWorldM() const noexcept { return position_world_m_; }
    [[nodiscard]] Eigen::Matrix3d positionCovarianceWorldM2() const noexcept {
        return position_covariance_world_m2_;
    }

private:
    GpsMeasurement(MeasurementAcquisition acquisition, const Vector3& position_world_m,
                   const Eigen::Matrix3d& covariance) noexcept;

    MeasurementAcquisition acquisition_;
    Vector3 position_world_m_;
    Eigen::Matrix3d position_covariance_world_m2_;
};

class ImuMeasurement {
public:
    [[nodiscard]] static std::optional<ImuMeasurement> create(
        MeasurementAcquisition acquisition, const Vector3& specific_force_body_mps2,
        const Vector3& angular_velocity_body_radps,
        const Eigen::Matrix3d& specific_force_covariance_body_m2ps4,
        const Eigen::Matrix3d& angular_velocity_covariance_body_rad2ps2);

    [[nodiscard]] const MeasurementAcquisition& acquisition() const noexcept { return acquisition_; }
    // Accelerometer specific force in body FLU. This differs from world linear acceleration.
    [[nodiscard]] Vector3 specificForceBodyMps2() const noexcept { return specific_force_body_mps2_; }
    [[nodiscard]] Vector3 angularVelocityBodyRadps() const noexcept {
        return angular_velocity_body_radps_;
    }
    [[nodiscard]] Eigen::Matrix3d specificForceCovarianceBodyM2ps4() const noexcept {
        return specific_force_covariance_body_m2ps4_;
    }
    [[nodiscard]] Eigen::Matrix3d angularVelocityCovarianceBodyRad2ps2() const noexcept {
        return angular_velocity_covariance_body_rad2ps2_;
    }

private:
    ImuMeasurement(MeasurementAcquisition acquisition, const Vector3& specific_force_body_mps2,
                   const Vector3& angular_velocity_body_radps,
                   const Eigen::Matrix3d& specific_force_covariance,
                   const Eigen::Matrix3d& angular_velocity_covariance) noexcept;

    MeasurementAcquisition acquisition_;
    Vector3 specific_force_body_mps2_;
    Vector3 angular_velocity_body_radps_;
    Eigen::Matrix3d specific_force_covariance_body_m2ps4_;
    Eigen::Matrix3d angular_velocity_covariance_body_rad2ps2_;
};

} // namespace aerotwin
