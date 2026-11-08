#pragma once

#include <aerotwin/sensors/GaussianNoise.hpp>
#include <aerotwin/sensors/Measurements.hpp>

#include <string>

namespace aerotwin {

class VehicleState;

// Linear acceleration of the vehicle reference point in ENU m/s².
class ImuKinematics {
public:
    [[nodiscard]] static std::optional<ImuKinematics> create(
        const Vector3& acceleration_world_mps2, SimulationTimestamp timestamp) noexcept;

    [[nodiscard]] Vector3 accelerationWorldMps2() const noexcept { return acceleration_; }
    [[nodiscard]] SimulationTimestamp timestamp() const noexcept { return timestamp_; }

private:
    ImuKinematics(const Vector3& acceleration, SimulationTimestamp timestamp) noexcept
        : acceleration_(acceleration), timestamp_(timestamp) {}

    Vector3 acceleration_;
    SimulationTimestamp timestamp_;
};

struct ImuNoiseSeeds {
    std::uint64_t accelerometer;
    std::uint64_t gyroscope;
};

class ImuSensorConfig {
public:
    // The two channel seeds must be distinct. Gravity is in world ENU m/s².
    [[nodiscard]] static std::optional<ImuSensorConfig> create(
        std::string_view sensor_id, const Eigen::Matrix3d& specific_force_covariance_body_m2ps4,
        const Eigen::Matrix3d& angular_velocity_covariance_body_rad2ps2, ImuNoiseSeeds seeds,
        const Vector3& gravity_world_mps2 = Vector3(0.0, 0.0, -9.80665));

    ImuSensorConfig(const ImuSensorConfig&) = default;
    ImuSensorConfig& operator=(const ImuSensorConfig&) = default;

    [[nodiscard]] std::string_view sensorId() const noexcept { return sensor_id_; }
    [[nodiscard]] ImuNoiseSeeds seeds() const noexcept { return seeds_; }
    [[nodiscard]] Vector3 gravityWorldMps2() const noexcept { return gravity_; }

private:
    friend class ImuSensor;

    ImuSensorConfig(const ImuMeasurement& validated, ImuNoiseSeeds seeds,
                    const Vector3& gravity, const GaussianNoise3& accelerometer_noise,
                    const GaussianNoise3& gyroscope_noise);

    std::string sensor_id_;
    ImuNoiseSeeds seeds_;
    Vector3 gravity_;
    Eigen::Matrix3d specific_force_covariance_;
    Eigen::Matrix3d angular_velocity_covariance_;
    GaussianNoise3 accelerometer_noise_;
    GaussianNoise3 gyroscope_noise_;
};

class ImuSensor {
public:
    explicit ImuSensor(const ImuSensorConfig& config);

    [[nodiscard]] std::string_view sensorId() const noexcept { return config_.sensorId(); }

    // Restarts both streams from their configured seeds.
    void reset() noexcept;

    // All three timestamps must match. Failures before sampling consume no draws.
    // Once sampling starts, both channels consume three normal draws each.
    [[nodiscard]] std::optional<ImuMeasurement> measure(
        const VehicleState& truth, const ImuKinematics& kinematics,
        SimulationTimestamp acquisition_timestamp);

private:
    ImuSensorConfig config_;
    GaussianNoise accelerometer_stream_;
    GaussianNoise gyroscope_stream_;
};

} // namespace aerotwin
