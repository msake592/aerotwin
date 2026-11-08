#include <aerotwin/sensors/GpsSensor.hpp>

#include <aerotwin/simulation/VehicleState.hpp>

namespace aerotwin {

std::optional<GpsSensorConfig> GpsSensorConfig::create(
    std::string_view sensor_id, const Eigen::Matrix3d& position_covariance_world_m2,
    std::uint64_t seed) {
    // Validate identity and obtain the canonical measurement covariance.
    const auto acquisition = MeasurementAcquisition::create(sensor_id, {});
    if (!acquisition) {
        return std::nullopt;
    }
    const auto measurement =
        GpsMeasurement::create(*acquisition, Vector3::Zero(), position_covariance_world_m2);
    if (!measurement) {
        return std::nullopt;
    }
    const auto covariance = measurement->positionCovarianceWorldM2();
    const auto noise = GaussianNoise3::create(covariance);
    if (!noise) {
        return std::nullopt;
    }
    return GpsSensorConfig(sensor_id, covariance, seed, *noise);
}

GpsSensorConfig::GpsSensorConfig(std::string_view sensor_id, const Eigen::Matrix3d& covariance,
                                 std::uint64_t seed, const GaussianNoise3& noise)
    : sensor_id_(sensor_id), covariance_(covariance), seed_(seed), noise_(noise) {}

GpsSensor::GpsSensor(const GpsSensorConfig& config) : config_(config), stream_(config.seed()) {}

void GpsSensor::reset() noexcept {
    stream_.reseed(config_.seed());
}

std::optional<GpsMeasurement> GpsSensor::measure(
    const VehicleState& truth, SimulationTimestamp acquisition_timestamp) {
    if (truth.timestamp() != acquisition_timestamp) {
        return std::nullopt;
    }
    const Vector3 position = truth.positionWorldM();
    const auto acquisition = MeasurementAcquisition::create(sensorId(), acquisition_timestamp);
    if (!acquisition || !isFinite(position)) {
        return std::nullopt;
    }

    const auto noise = config_.noise_.sample(stream_);
    if (!noise) {
        return std::nullopt;
    }
    // Preserve the exact snapshot (including signed zero) for a zero draw.
    Vector3 measured_position = position;
    if ((noise->array() != 0.0).any()) {
        measured_position += *noise;
    }
    return GpsMeasurement::create(*acquisition, measured_position, config_.covariance_);
}

} // namespace aerotwin
