#pragma once

#include <aerotwin/sensors/GaussianNoise.hpp>
#include <aerotwin/sensors/SensorModel.hpp>

#include <string>

namespace aerotwin {

class GpsSensorConfig {
public:
    [[nodiscard]] static std::optional<GpsSensorConfig> create(
        std::string_view sensor_id, const Eigen::Matrix3d& position_covariance_world_m2,
        std::uint64_t seed);

    GpsSensorConfig(const GpsSensorConfig&) = default;
    GpsSensorConfig& operator=(const GpsSensorConfig&) = default;

    [[nodiscard]] std::string_view sensorId() const noexcept { return sensor_id_; }
    [[nodiscard]] Eigen::Matrix3d positionCovarianceWorldM2() const noexcept { return covariance_; }
    [[nodiscard]] std::uint64_t seed() const noexcept { return seed_; }

private:
    friend class GpsSensor;

    GpsSensorConfig(std::string_view sensor_id, const Eigen::Matrix3d& covariance,
                    std::uint64_t seed, const GaussianNoise3& noise);

    std::string sensor_id_;
    Eigen::Matrix3d covariance_;
    std::uint64_t seed_;
    GaussianNoise3 noise_;
};

class GpsSensor final : public SensorModel<GpsMeasurement> {
public:
    explicit GpsSensor(const GpsSensorConfig& config);

    [[nodiscard]] std::string_view sensorId() const noexcept override { return config_.sensorId(); }

    // Restart from the configured seed. Clock resets do not reset the stream.
    void reset() noexcept;

    // A timestamp mismatch consumes no draws. Failures after sampling keep
    // the three normal draws consumed by the M2.2 sampler.
    [[nodiscard]] std::optional<GpsMeasurement> measure(
        const VehicleState& truth, SimulationTimestamp acquisition_timestamp) override;

private:
    GpsSensorConfig config_;
    GaussianNoise stream_;
};

} // namespace aerotwin
