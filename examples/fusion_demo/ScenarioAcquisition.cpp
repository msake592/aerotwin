#include "ScenarioAcquisition.hpp"

#include <utility>

namespace aerotwin::fusion_demo {

AcquisitionResult acquireScenario(SimulationNanoseconds gps_period, ImuNoiseSeeds imu_seeds) {
    constexpr SimulationNanoseconds tick{10'000'000};
    constexpr std::uint64_t gps_seed = 42;
    const auto initial = VehicleState::create(Vector3(10, -5, 2), Quaternion::Identity(),
                                              Vector3(2, 1, 0.5), Vector3::Zero(), {});
    if (!initial) {
        return AcquisitionFailure{AcquisitionError::ScenarioConstruction};
    }
    const auto scenario_config = ScenarioConfig::create(*initial, tick, 400, 42, "m2_fusion_demo");
    if (!scenario_config) {
        return AcquisitionFailure{AcquisitionError::ScenarioConstruction};
    }
    auto scenario = runScenario(*scenario_config);
    if (!scenario) {
        return AcquisitionFailure{AcquisitionError::ScenarioExecution};
    }

    const auto gps = GpsSensorConfig::create("gps", Eigen::Matrix3d::Identity(), gps_seed);
    const auto imu = ImuSensorConfig::create("imu", 0.01 * Eigen::Matrix3d::Identity(),
                                             0.0001 * Eigen::Matrix3d::Identity(), imu_seeds);
    if (!gps || !imu) {
        return AcquisitionFailure{AcquisitionError::SensorConfiguration};
    }
    auto pipeline = SensorSamplingPipeline::create(GpsSamplingConfig{*gps, gps_period},
                                                    ImuSamplingConfig{*imu, tick});
    if (!pipeline) {
        return AcquisitionFailure{AcquisitionError::SensorConfiguration};
    }

    AcquisitionHistory history{std::move(*scenario), {}, 0, 0, gps_period, gps_seed,
                               imu->seeds(), imu->gravityWorldMps2()};
    history.batches.reserve(history.scenario.samples().size());
    for (const auto& truth : history.scenario.samples()) {
        const auto timestamp = truth.timestamp();
        const auto kinematics = ImuKinematics::create(Vector3::Zero(), timestamp);
        if (!kinematics) {
            return AcquisitionFailure{AcquisitionError::ImuKinematics, timestamp};
        }
        auto result = pipeline->sample(truth, timestamp, &*kinematics);
        if (const auto* error = std::get_if<SamplingError>(&result)) {
            return AcquisitionFailure{AcquisitionError::Sampling, timestamp, *error};
        }
        auto& batch = std::get<MeasurementBatch>(result);
        // The IMU is due at every tick. The only possible shapes are IMU
        // alone or GPS followed by IMU, in contiguous slots.
        const bool has_gps = batch[0] && std::holds_alternative<GpsMeasurement>(*batch[0]);
        const auto& imu_slot = batch[has_gps ? 1 : 0];
        if (!imu_slot || !std::holds_alternative<ImuMeasurement>(*imu_slot) ||
            (!has_gps && batch[1])) {
            return AcquisitionFailure{AcquisitionError::AcquisitionOrder, timestamp};
        }
        if (std::get<ImuMeasurement>(*imu_slot).acquisition().timestamp() != timestamp ||
            (has_gps && std::get<GpsMeasurement>(*batch[0]).acquisition().timestamp() != timestamp)) {
            return AcquisitionFailure{AcquisitionError::TimestampMismatch, timestamp};
        }

        history.gps_count += has_gps;
        ++history.imu_count;
        history.batches.push_back(std::move(batch));
    }
    return history;
}

} // namespace aerotwin::fusion_demo
