#pragma once

#include <aerotwin/sensors/GpsSensor.hpp>
#include <aerotwin/sensors/ImuSensor.hpp>

#include <array>
#include <variant>

namespace aerotwin {

struct GpsSamplingConfig {
    GpsSensorConfig sensor;
    SimulationNanoseconds period;
};

struct ImuSamplingConfig {
    ImuSensorConfig sensor;
    SimulationNanoseconds period;
};

using SensorMeasurement = std::variant<GpsMeasurement, ImuMeasurement>;
// Occupied entries are contiguous, in GPS-before-IMU order when both are due.
using MeasurementBatch = std::array<std::optional<SensorMeasurement>, 2>;

enum class SamplingError {
    TimestampMismatch,
    EpochMismatch,
    DuplicateTimestamp,
    NonmonotonicTimestamp,
    MissedSample,
    MissingImuKinematics,
    ImuKinematicsTimestampMismatch,
    GpsMeasurementFailed,
    ImuMeasurementFailed,
};

using SamplingResult = std::variant<MeasurementBatch, SamplingError>;

class SensorSamplingPipeline {
public:
    // At least one sensor must be present. Configurations are already validated.
    [[nodiscard]] static std::optional<SensorSamplingPipeline> create(
        const std::optional<GpsSamplingConfig>& gps, const std::optional<ImuSamplingConfig>& imu);

    // Starts again at elapsed zero in the supplied epoch and restarts both streams.
    void reset(std::uint64_t epoch) noexcept;

    // Only due IMU samples require kinematics. A returned error commits no state.
    [[nodiscard]] SamplingResult sample(const VehicleState& truth, SimulationTimestamp timestamp,
                                        const ImuKinematics* kinematics = nullptr);

private:
    SensorSamplingPipeline(const std::optional<GpsSamplingConfig>& gps,
                           const std::optional<ImuSamplingConfig>& imu);

    std::optional<GpsSensor> gps_;
    std::optional<ImuSensor> imu_;
    SimulationNanoseconds gps_period_{};
    SimulationNanoseconds imu_period_{};
    std::uint64_t epoch_{};
    std::optional<SimulationNanoseconds> last_elapsed_;
};

} // namespace aerotwin
