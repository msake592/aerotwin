#include <aerotwin/sensors/ImuSensor.hpp>

#include <aerotwin/math/Transformations.hpp>
#include <aerotwin/simulation/VehicleState.hpp>

namespace aerotwin {
namespace {

Vector3 addNoise(const Vector3& value, const Vector3& noise) {
    if ((noise.array() == 0.0).all()) {
        return value;
    }
    return value + noise;
}

} // namespace

std::optional<ImuKinematics> ImuKinematics::create(
    const Vector3& acceleration_world_mps2, SimulationTimestamp timestamp) noexcept {
    if (!isFinite(acceleration_world_mps2) || timestamp.elapsed < SimulationNanoseconds::zero()) {
        return std::nullopt;
    }
    return ImuKinematics(acceleration_world_mps2, timestamp);
}

std::optional<ImuSensorConfig> ImuSensorConfig::create(
    std::string_view sensor_id, const Eigen::Matrix3d& specific_force_covariance_body_m2ps4,
    const Eigen::Matrix3d& angular_velocity_covariance_body_rad2ps2, ImuNoiseSeeds seeds,
    const Vector3& gravity_world_mps2) {
    const auto acquisition = MeasurementAcquisition::create(sensor_id, {});
    if (!acquisition || !isFinite(gravity_world_mps2) || seeds.accelerometer == seeds.gyroscope) {
        return std::nullopt;
    }
    const auto validated = ImuMeasurement::create(
        *acquisition, Vector3::Zero(), Vector3::Zero(), specific_force_covariance_body_m2ps4,
        angular_velocity_covariance_body_rad2ps2);
    if (!validated) {
        return std::nullopt;
    }

    const auto accelerometer_noise =
        GaussianNoise3::create(validated->specificForceCovarianceBodyM2ps4());
    const auto gyroscope_noise =
        GaussianNoise3::create(validated->angularVelocityCovarianceBodyRad2ps2());
    if (!accelerometer_noise || !gyroscope_noise) {
        return std::nullopt;
    }
    return ImuSensorConfig(*validated, seeds, gravity_world_mps2, *accelerometer_noise,
                           *gyroscope_noise);
}

ImuSensorConfig::ImuSensorConfig(const ImuMeasurement& validated, ImuNoiseSeeds seeds,
                                 const Vector3& gravity, const GaussianNoise3& accelerometer_noise,
                                 const GaussianNoise3& gyroscope_noise)
    : sensor_id_(validated.acquisition().sensorId()), seeds_(seeds), gravity_(gravity),
      specific_force_covariance_(validated.specificForceCovarianceBodyM2ps4()),
      angular_velocity_covariance_(validated.angularVelocityCovarianceBodyRad2ps2()),
      accelerometer_noise_(accelerometer_noise), gyroscope_noise_(gyroscope_noise) {}

ImuSensor::ImuSensor(const ImuSensorConfig& config)
    : config_(config), accelerometer_stream_(config.seeds().accelerometer),
      gyroscope_stream_(config.seeds().gyroscope) {}

void ImuSensor::reset() noexcept {
    accelerometer_stream_.reseed(config_.seeds().accelerometer);
    gyroscope_stream_.reseed(config_.seeds().gyroscope);
}

std::optional<ImuMeasurement> ImuSensor::measure(
    const VehicleState& truth, const ImuKinematics& kinematics,
    SimulationTimestamp acquisition_timestamp) {
    if (truth.timestamp() != acquisition_timestamp ||
        kinematics.timestamp() != acquisition_timestamp) {
        return std::nullopt;
    }
    const auto acquisition = MeasurementAcquisition::create(sensorId(), acquisition_timestamp);
    if (!acquisition) {
        return std::nullopt;
    }

    const Vector3 force_world = kinematics.accelerationWorldMps2() - config_.gravityWorldMps2();
    const auto force_body = worldToBodyVector(truth.orientationWorldFromBody(), force_world);
    const Vector3 rate_body = truth.angularVelocityBodyRadps();
    if (!force_body || !isFinite(rate_body)) {
        return std::nullopt;
    }

    const auto force_noise = config_.accelerometer_noise_.sample(accelerometer_stream_);
    const auto rate_noise = config_.gyroscope_noise_.sample(gyroscope_stream_);
    if (!force_noise || !rate_noise) {
        return std::nullopt;
    }
    return ImuMeasurement::create(*acquisition, addNoise(*force_body, *force_noise),
                                  addNoise(rate_body, *rate_noise),
                                  config_.specific_force_covariance_,
                                  config_.angular_velocity_covariance_);
}

} // namespace aerotwin
