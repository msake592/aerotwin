#include <aerotwin/sensors/SensorSamplingPipeline.hpp>
#include <aerotwin/simulation/VehicleState.hpp>

#include <gtest/gtest.h>

#include <limits>

namespace aerotwin {
namespace {

const Vector3 zero = Vector3::Zero();
const Eigen::Matrix3d gps_covariance = Eigen::Matrix3d::Identity();
const Eigen::Matrix3d force_covariance = 2.0 * Eigen::Matrix3d::Identity();
const Eigen::Matrix3d rate_covariance = 3.0 * Eigen::Matrix3d::Identity();
constexpr SimulationNanoseconds gps_period{1'000'000'000};
constexpr SimulationNanoseconds imu_period{10'000'000};

GpsSamplingConfig gpsConfig(SimulationNanoseconds period = gps_period) {
    return {GpsSensorConfig::create("gps", gps_covariance, 100).value(), period};
}

ImuSamplingConfig imuConfig(SimulationNanoseconds period = imu_period,
                            const Vector3& gravity = zero) {
    return {ImuSensorConfig::create("imu", force_covariance, rate_covariance, {101, 102}, gravity)
                .value(),
            period};
}

VehicleState truthAt(SimulationTimestamp timestamp) {
    return VehicleState::create(Vector3(1.0, 2.0, 3.0), Quaternion::Identity(), zero, zero,
                                timestamp).value();
}

SamplingResult sampleAt(SensorSamplingPipeline& pipeline, SimulationTimestamp timestamp,
                        const ImuKinematics* kinematics = nullptr) {
    return pipeline.sample(truthAt(timestamp), timestamp, kinematics);
}

void expectError(const SamplingResult& result, SamplingError expected) {
    const auto* error = std::get_if<SamplingError>(&result);
    ASSERT_NE(error, nullptr);
    EXPECT_EQ(*error, expected);
}

void expectSameBatch(const SamplingResult& actual, const SamplingResult& expected) {
    const auto* left = std::get_if<MeasurementBatch>(&actual);
    const auto* right = std::get_if<MeasurementBatch>(&expected);
    ASSERT_NE(left, nullptr);
    ASSERT_NE(right, nullptr);
    for (std::size_t index = 0; index < left->size(); ++index) {
        ASSERT_EQ((*left)[index].has_value(), (*right)[index].has_value());
        if (!(*left)[index]) {
            continue;
        }
        const auto& a = *(*left)[index];
        const auto& b = *(*right)[index];
        ASSERT_EQ(a.index(), b.index());
        if (const auto* gps = std::get_if<GpsMeasurement>(&a)) {
            const auto& other = std::get<GpsMeasurement>(b);
            EXPECT_EQ(gps->positionWorldM(), other.positionWorldM());
            EXPECT_EQ(gps->positionCovarianceWorldM2(), other.positionCovarianceWorldM2());
            EXPECT_EQ(gps->acquisition().sensorId(), other.acquisition().sensorId());
            EXPECT_EQ(gps->acquisition().timestamp(), other.acquisition().timestamp());
        } else {
            const auto& imu = std::get<ImuMeasurement>(a);
            const auto& other = std::get<ImuMeasurement>(b);
            EXPECT_EQ(imu.specificForceBodyMps2(), other.specificForceBodyMps2());
            EXPECT_EQ(imu.angularVelocityBodyRadps(), other.angularVelocityBodyRadps());
            EXPECT_EQ(imu.specificForceCovarianceBodyM2ps4(),
                      other.specificForceCovarianceBodyM2ps4());
            EXPECT_EQ(imu.angularVelocityCovarianceBodyRad2ps2(),
                      other.angularVelocityCovarianceBodyRad2ps2());
            EXPECT_EQ(imu.acquisition().sensorId(), other.acquisition().sensorId());
            EXPECT_EQ(imu.acquisition().timestamp(), other.acquisition().timestamp());
        }
    }
}

TEST(SensorSamplingPipelineTest, RequiresAtLeastOneSensorAndPositivePeriods) {
    EXPECT_FALSE(SensorSamplingPipeline::create(std::nullopt, std::nullopt));
    EXPECT_FALSE(SensorSamplingPipeline::create(gpsConfig({}), imuConfig()));
    EXPECT_FALSE(SensorSamplingPipeline::create(gpsConfig(), imuConfig(SimulationNanoseconds{-1})));
    EXPECT_TRUE(SensorSamplingPipeline::create(gpsConfig(), imuConfig()));
}

TEST(SensorSamplingPipelineTest, GpsOnlyStartsAtZeroAndRejectsSkippedDueTimes) {
    auto pipeline = SensorSamplingPipeline::create(gpsConfig(), std::nullopt);
    ASSERT_TRUE(pipeline);
    expectError(sampleAt(*pipeline, {0, SimulationNanoseconds{1}}), SamplingError::MissedSample);

    const auto initial = sampleAt(*pipeline, {});
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(initial));
    const auto& batch = std::get<MeasurementBatch>(initial);
    ASSERT_TRUE(batch[0]);
    EXPECT_TRUE(std::holds_alternative<GpsMeasurement>(*batch[0]));
    EXPECT_FALSE(batch[1]);

    const SimulationTimestamp between{0, gps_period / 2};
    const auto idle = sampleAt(*pipeline, between);
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(idle));
    EXPECT_FALSE(std::get<MeasurementBatch>(idle)[0]);
    expectError(sampleAt(*pipeline, between), SamplingError::DuplicateTimestamp);
    expectError(sampleAt(*pipeline, {}), SamplingError::NonmonotonicTimestamp);
    expectError(sampleAt(*pipeline, {0, 2 * gps_period}), SamplingError::MissedSample);
    expectError(pipeline->sample(truthAt(between), {0, gps_period}), SamplingError::TimestampMismatch);

    const auto due = sampleAt(*pipeline, {0, gps_period});
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(due));
    ASSERT_TRUE(std::get<MeasurementBatch>(due)[0]);
    EXPECT_TRUE(std::holds_alternative<GpsMeasurement>(*std::get<MeasurementBatch>(due)[0]));
}

TEST(SensorSamplingPipelineTest, ImuOnlyRequiresKinematicsOnlyAtDueTimes) {
    auto pipeline = SensorSamplingPipeline::create(std::nullopt, imuConfig());
    ASSERT_TRUE(pipeline);
    const auto kinematics = ImuKinematics::create(zero, {}).value();
    expectError(sampleAt(*pipeline, {}), SamplingError::MissingImuKinematics);

    const auto initial = sampleAt(*pipeline, {}, &kinematics);
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(initial));
    const auto& batch = std::get<MeasurementBatch>(initial);
    ASSERT_TRUE(batch[0]);
    EXPECT_TRUE(std::holds_alternative<ImuMeasurement>(*batch[0]));
    EXPECT_FALSE(batch[1]);

    const auto idle = sampleAt(*pipeline, {0, imu_period / 2});
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(idle));
    EXPECT_FALSE(std::get<MeasurementBatch>(idle)[0]);
    expectError(sampleAt(*pipeline, {0, imu_period}, &kinematics),
                SamplingError::ImuKinematicsTimestampMismatch);
    expectError(sampleAt(*pipeline, {0, imu_period + SimulationNanoseconds{1}}),
                SamplingError::MissedSample);
}

TEST(SensorSamplingPipelineTest, SamplesStationaryTruthAtOneAndOneHundredHertz) {
    const auto gps = gpsConfig();
    const auto imu = imuConfig();
    auto pipeline = SensorSamplingPipeline::create(gps, imu);
    auto replay = SensorSamplingPipeline::create(gps, imu);
    ASSERT_TRUE(pipeline);
    ASSERT_TRUE(replay);
    GpsSensor gps_reference(gps.sensor);
    ImuSensor imu_reference(imu.sensor);
    int gps_count = 0;
    int imu_count = 0;

    // Inclusive interval [0 s, 2 s]. Stationary truth has zero rate and acceleration.
    for (int tick = 0; tick <= 200; ++tick) {
        const SimulationTimestamp timestamp{0, tick * imu_period};
        const auto truth = truthAt(timestamp);
        const auto kinematics = ImuKinematics::create(zero, timestamp).value();
        const auto result = pipeline->sample(truth, timestamp, &kinematics);
        expectSameBatch(result, replay->sample(truth, timestamp, &kinematics));
        ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(result));
        const auto& batch = std::get<MeasurementBatch>(result);
        const bool gps_due = tick % 100 == 0;
        MeasurementBatch expected{};
        if (gps_due) {
            const auto measured = gps_reference.measure(truth, timestamp);
            ASSERT_TRUE(measured);
            expected[0].emplace(*measured);
            ASSERT_TRUE(batch[0]);
            ASSERT_TRUE(std::holds_alternative<GpsMeasurement>(*batch[0]));
            const auto& gps_sample = std::get<GpsMeasurement>(*batch[0]);
            EXPECT_EQ(gps_sample.acquisition().timestamp(), timestamp);
            EXPECT_EQ(gps_sample.positionCovarianceWorldM2(), gps_covariance);
            ++gps_count;
        }
        const auto measured = imu_reference.measure(truth, kinematics, timestamp);
        ASSERT_TRUE(measured);
        const auto index = gps_due ? 1 : 0;
        expected[index].emplace(*measured);
        ASSERT_TRUE(batch[index]);
        ASSERT_TRUE(std::holds_alternative<ImuMeasurement>(*batch[index]));
        const auto& imu_sample = std::get<ImuMeasurement>(*batch[index]);
        EXPECT_EQ(imu_sample.acquisition().timestamp(), timestamp);
        EXPECT_EQ(imu_sample.specificForceCovarianceBodyM2ps4(), force_covariance);
        EXPECT_EQ(imu_sample.angularVelocityCovarianceBodyRad2ps2(), rate_covariance);
        ++imu_count;
        expectSameBatch(result, expected);
    }
    EXPECT_EQ(gps_count, 3);
    EXPECT_EQ(imu_count, 201);
}

TEST(SensorSamplingPipelineTest, CopySnapshotsScheduleAndAllNoiseStreams) {
    const auto gps = gpsConfig(2 * imu_period);
    const auto imu = imuConfig();
    auto pipeline = SensorSamplingPipeline::create(gps, imu);
    ASSERT_TRUE(pipeline);
    const auto sample_tick = [](SensorSamplingPipeline& sampler, int tick) {
        const SimulationTimestamp timestamp{0, tick * imu_period};
        const auto kinematics = ImuKinematics::create(zero, timestamp).value();
        return sampleAt(sampler, timestamp, &kinematics);
    };

    const auto initial = sample_tick(*pipeline, 0);
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(initial));
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(sample_tick(*pipeline, 1)));
    SensorSamplingPipeline copy(*pipeline);

    // Advance the original through another tie and an IMU-only tick before
    // sampling the copy from its saved position.
    const auto tie = sample_tick(*pipeline, 2);
    const auto imu_only = sample_tick(*pipeline, 3);
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(tie));
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(imu_only));
    const auto& first_batch = std::get<MeasurementBatch>(initial);
    const auto& tie_batch = std::get<MeasurementBatch>(tie);
    ASSERT_TRUE(first_batch[0] && first_batch[1] && tie_batch[0] && tie_batch[1]);
    ASSERT_TRUE(std::holds_alternative<GpsMeasurement>(*tie_batch[0]));
    ASSERT_TRUE(std::holds_alternative<ImuMeasurement>(*tie_batch[1]));
    const auto& initial_gps = std::get<GpsMeasurement>(*first_batch[0]);
    const auto& initial_imu = std::get<ImuMeasurement>(*first_batch[1]);
    const auto& tie_gps = std::get<GpsMeasurement>(*tie_batch[0]);
    const auto& tie_imu = std::get<ImuMeasurement>(*tie_batch[1]);
    EXPECT_NE(tie_gps.positionWorldM(), initial_gps.positionWorldM());
    EXPECT_NE(tie_imu.specificForceBodyMps2(), initial_imu.specificForceBodyMps2());
    EXPECT_NE(tie_imu.angularVelocityBodyRadps(), initial_imu.angularVelocityBodyRadps());

    expectError(sample_tick(copy, 1), SamplingError::DuplicateTimestamp);
    expectSameBatch(sample_tick(copy, 2), tie);
    expectSameBatch(sample_tick(copy, 3), imu_only);
    const auto copy_next = sample_tick(copy, 4);
    const auto original_next = sample_tick(*pipeline, 4);
    expectSameBatch(copy_next, original_next);
}

TEST(SensorSamplingPipelineTest, ExplicitResetRestartsScheduleAndBothStreams) {
    auto pipeline = SensorSamplingPipeline::create(gpsConfig(), imuConfig());
    ASSERT_TRUE(pipeline);
    const auto kinematics = ImuKinematics::create(zero, {}).value();
    const auto initial = sampleAt(*pipeline, {}, &kinematics);
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(initial));
    const SimulationTimestamp new_epoch{5, {}};
    const auto new_kinematics = ImuKinematics::create(zero, new_epoch).value();
    expectError(sampleAt(*pipeline, new_epoch, &new_kinematics), SamplingError::EpochMismatch);

    pipeline->reset(5);
    expectError(sampleAt(*pipeline, {}, &kinematics), SamplingError::EpochMismatch);
    expectError(sampleAt(*pipeline, {5, imu_period}), SamplingError::MissedSample);
    const auto restarted = sampleAt(*pipeline, new_epoch, &new_kinematics);
    auto replay = SensorSamplingPipeline::create(gpsConfig(), imuConfig());
    ASSERT_TRUE(replay);
    replay->reset(5);
    expectSameBatch(restarted, sampleAt(*replay, new_epoch, &new_kinematics));
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(restarted));
    const auto& batch = std::get<MeasurementBatch>(restarted);
    ASSERT_TRUE(batch[0]);
    ASSERT_TRUE(batch[1]);
    EXPECT_EQ(std::get<GpsMeasurement>(*batch[0]).acquisition().timestamp(), new_epoch);
    EXPECT_EQ(std::get<ImuMeasurement>(*batch[1]).acquisition().timestamp(), new_epoch);

    pipeline->reset(0);
    expectSameBatch(sampleAt(*pipeline, {}, &kinematics), initial);
}

TEST(SensorSamplingPipelineTest, FailedImuDoesNotCommitDueGpsOrAdvanceEitherSensor) {
    const double max = std::numeric_limits<double>::max();
    const Vector3 gravity(max, 0.0, 0.0);
    const auto imu = imuConfig(gps_period, gravity);
    auto pipeline = SensorSamplingPipeline::create(gpsConfig(), imu);
    auto reference = SensorSamplingPipeline::create(gpsConfig(), imu);
    ASSERT_TRUE(pipeline);
    ASSERT_TRUE(reference);
    const auto initial = ImuKinematics::create(gravity, {}).value();
    expectSameBatch(sampleAt(*pipeline, {}, &initial), sampleAt(*reference, {}, &initial));

    const SimulationTimestamp due{0, gps_period};
    const auto extreme = ImuKinematics::create(-gravity, due).value();
    expectError(sampleAt(*pipeline, due), SamplingError::MissingImuKinematics);
    expectError(sampleAt(*pipeline, due, &initial), SamplingError::ImuKinematicsTimestampMismatch);
    expectError(sampleAt(*pipeline, due, &extreme), SamplingError::ImuMeasurementFailed);
    const auto valid = ImuKinematics::create(gravity, due).value();
    expectSameBatch(sampleAt(*pipeline, due, &valid), sampleAt(*reference, due, &valid));

    const SimulationTimestamp later{0, 2 * gps_period};
    const auto later_kinematics = ImuKinematics::create(gravity, later).value();
    expectSameBatch(sampleAt(*pipeline, later, &later_kinematics),
                    sampleAt(*reference, later, &later_kinematics));
}

TEST(SensorSamplingPipelineTest, SchedulesAtIntegerLimitWithoutWrapping) {
    const auto max = SimulationNanoseconds::max();
    auto pipeline = SensorSamplingPipeline::create(gpsConfig(max), std::nullopt);
    ASSERT_TRUE(pipeline);
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(sampleAt(*pipeline, {})));
    const auto before = sampleAt(*pipeline, {0, max - SimulationNanoseconds{1}});
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(before));
    EXPECT_FALSE(std::get<MeasurementBatch>(before)[0]);
    const auto final = sampleAt(*pipeline, {0, max});
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(final));
    ASSERT_TRUE(std::get<MeasurementBatch>(final)[0]);
    expectError(sampleAt(*pipeline, {0, max}), SamplingError::DuplicateTimestamp);

    auto exhausted = SensorSamplingPipeline::create(
        gpsConfig(max - SimulationNanoseconds{1}), std::nullopt);
    ASSERT_TRUE(exhausted);
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(sampleAt(*exhausted, {})));
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(
        sampleAt(*exhausted, {0, max - SimulationNanoseconds{1}})));
    const auto idle = sampleAt(*exhausted, {0, max});
    ASSERT_TRUE(std::holds_alternative<MeasurementBatch>(idle));
    EXPECT_FALSE(std::get<MeasurementBatch>(idle)[0]);
}

} // namespace
} // namespace aerotwin
