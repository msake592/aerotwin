#include "ScenarioAcquisition.hpp"

#include <gtest/gtest.h>

#include <type_traits>

namespace aerotwin::fusion_demo {
namespace {

using Time = SimulationNanoseconds;

TEST(FusionAcquisitionTest, OwnsAnalyticalTruthAndScheduledObservationsInSlotOrder) {
    const auto result = acquireScenario();
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(result));
    const auto& history = std::get<AcquisitionHistory>(result);
    const auto& truth = history.scenario.samples();
    ASSERT_EQ(truth.size(), 401u);
    ASSERT_EQ(history.batches.size(), truth.size());
    EXPECT_EQ(history.gps_count, 5u);
    EXPECT_EQ(history.imu_count, 401u);
    EXPECT_EQ(truth.front().positionWorldM(), Vector3(10, -5, 2));
    EXPECT_TRUE(truth.back().positionWorldM().isApprox(Vector3(18, -1, 4), 1e-12));

    for (std::size_t i = 0; i < truth.size(); ++i) {
        SCOPED_TRACE(i);
        const SimulationTimestamp expected_time{0, Time{10'000'000 * static_cast<Time::rep>(i)}};
        EXPECT_EQ(truth[i].timestamp(), expected_time);
        EXPECT_EQ(truth[i].linearVelocityWorldMps(), Vector3(2, 1, 0.5));
        EXPECT_EQ(truth[i].orientationWorldFromBody().coeffs(), Quaternion::Identity().coeffs());
        EXPECT_EQ(truth[i].angularVelocityBodyRadps(), Vector3::Zero());
        const double seconds = static_cast<double>(i) / 100.0;
        const Vector3 expected_position(10 + 2 * seconds, -5 + seconds, 2 + 0.5 * seconds);
        EXPECT_TRUE(truth[i].positionWorldM().isApprox(expected_position, 1e-12));

        const auto& batch = history.batches[i];
        const bool gps_due = i % 100 == 0;
        if (gps_due) {
            ASSERT_TRUE(batch[0] && std::holds_alternative<GpsMeasurement>(*batch[0]));
            const auto& gps = std::get<GpsMeasurement>(*batch[0]);
            EXPECT_EQ(gps.acquisition().timestamp(), expected_time);
            EXPECT_EQ(gps.acquisition().sensorId(), "gps");
            EXPECT_EQ(gps.positionCovarianceWorldM2(), Eigen::Matrix3d::Identity());
        } else {
            EXPECT_FALSE(batch[1]);
        }
        const auto& imu_slot = batch[gps_due ? 1 : 0];
        ASSERT_TRUE(imu_slot && std::holds_alternative<ImuMeasurement>(*imu_slot));
        const auto& imu = std::get<ImuMeasurement>(*imu_slot);
        EXPECT_EQ(imu.acquisition().timestamp(), expected_time);
        EXPECT_EQ(imu.acquisition().sensorId(), "imu");
        EXPECT_EQ(imu.specificForceCovarianceBodyM2ps4(), 0.01 * Eigen::Matrix3d::Identity());
        EXPECT_EQ(imu.angularVelocityCovarianceBodyRad2ps2(), 0.0001 * Eigen::Matrix3d::Identity());
    }

    // Isolate the ideal measurement with a separate noiseless IMU. The
    // acquisition history above contains the configured noisy measurements.
    const auto config = ImuSensorConfig::create("ideal", Eigen::Matrix3d::Zero(),
                                                Eigen::Matrix3d::Zero(), {43, 44});
    ASSERT_TRUE(config);
    ImuSensor ideal(*config);
    const auto timestamp = truth.back().timestamp();
    const auto kinematics = ImuKinematics::create(Vector3::Zero(), timestamp);
    ASSERT_TRUE(kinematics);
    const auto measurement = ideal.measure(truth.back(), *kinematics, timestamp);
    ASSERT_TRUE(measurement);
    EXPECT_EQ(measurement->specificForceBodyMps2(), Vector3(0, 0, 9.80665));
    EXPECT_EQ(measurement->angularVelocityBodyRadps(), Vector3::Zero());
}

TEST(FusionAcquisitionTest, FreshRunsReplayEverySnapshotAndMeasurementExactly) {
    const auto first = acquireScenario();
    const auto replay = acquireScenario();
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(first));
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(replay));
    const auto& a = std::get<AcquisitionHistory>(first);
    const auto& b = std::get<AcquisitionHistory>(replay);
    ASSERT_EQ(a.scenario.samples().size(), b.scenario.samples().size());
    ASSERT_EQ(a.batches.size(), b.batches.size());
    EXPECT_EQ(a.gps_count, b.gps_count);
    EXPECT_EQ(a.imu_count, b.imu_count);

    for (std::size_t i = 0; i < a.batches.size(); ++i) {
        SCOPED_TRACE(i);
        const auto& x = a.scenario.samples()[i];
        const auto& y = b.scenario.samples()[i];
        EXPECT_EQ(x.timestamp(), y.timestamp());
        EXPECT_EQ(x.positionWorldM(), y.positionWorldM());
        EXPECT_EQ(x.linearVelocityWorldMps(), y.linearVelocityWorldMps());
        EXPECT_EQ(x.orientationWorldFromBody().coeffs(), y.orientationWorldFromBody().coeffs());
        EXPECT_EQ(x.angularVelocityBodyRadps(), y.angularVelocityBodyRadps());
        for (std::size_t slot = 0; slot < a.batches[i].size(); ++slot) {
            const auto& left = a.batches[i][slot];
            const auto& right = b.batches[i][slot];
            ASSERT_EQ(left.has_value(), right.has_value());
            if (!left) {
                continue;
            }
            ASSERT_EQ(left->index(), right->index());
            std::visit([&](const auto& m) {
                const auto& n = std::get<std::decay_t<decltype(m)>>(*right);
                EXPECT_EQ(m.acquisition().timestamp(), n.acquisition().timestamp());
                EXPECT_EQ(m.acquisition().sensorId(), n.acquisition().sensorId());
                if constexpr (std::is_same_v<std::decay_t<decltype(m)>, GpsMeasurement>) {
                    EXPECT_EQ(m.positionWorldM(), n.positionWorldM());
                    EXPECT_EQ(m.positionCovarianceWorldM2(), n.positionCovarianceWorldM2());
                } else {
                    EXPECT_EQ(m.specificForceBodyMps2(), n.specificForceBodyMps2());
                    EXPECT_EQ(m.angularVelocityBodyRadps(), n.angularVelocityBodyRadps());
                    EXPECT_EQ(m.specificForceCovarianceBodyM2ps4(), n.specificForceCovarianceBodyM2ps4());
                    EXPECT_EQ(m.angularVelocityCovarianceBodyRad2ps2(),
                              n.angularVelocityCovarianceBodyRad2ps2());
                }
            }, *left);
        }
    }
}

TEST(FusionAcquisitionTest, ReturnsConfigurationAndOriginalMissedDeadlineErrors) {
    const auto invalid = acquireScenario(Time::zero());
    ASSERT_TRUE(std::holds_alternative<AcquisitionFailure>(invalid));
    const auto& setup = std::get<AcquisitionFailure>(invalid);
    EXPECT_EQ(setup.error, AcquisitionError::SensorConfiguration);
    EXPECT_FALSE(setup.timestamp);
    EXPECT_FALSE(setup.sampling_error);

    // The 15 ms GPS deadline lies between the 10 and 20 ms truth snapshots.
    const auto missed = acquireScenario(Time{15'000'000});
    ASSERT_TRUE(std::holds_alternative<AcquisitionFailure>(missed));
    const auto& failure = std::get<AcquisitionFailure>(missed);
    EXPECT_EQ(failure.error, AcquisitionError::Sampling);
    EXPECT_EQ(failure.timestamp, (SimulationTimestamp{0, Time{20'000'000}}));
    EXPECT_EQ(failure.sampling_error, SamplingError::MissedSample);
}

} // namespace
} // namespace aerotwin::fusion_demo
