#include <aerotwin/sensors/GpsSensor.hpp>
#include <aerotwin/simulation/VehicleState.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <type_traits>

namespace aerotwin {
namespace {

constexpr SimulationTimestamp capture{7, SimulationNanoseconds{123'456}};
constexpr std::uint64_t seed = 0xfedcba9876543210ULL;
const Vector3 zero = Vector3::Zero();
const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

static_assert(!std::is_default_constructible_v<GpsSensorConfig>);

std::optional<VehicleState> truthAt(const Vector3& position, SimulationTimestamp timestamp = capture) {
    return VehicleState::create(position, Quaternion::Identity(), zero, zero, timestamp);
}

TEST(GpsSensorTest, ConfigOwnsValidatedIdentitySeedAndCanonicalCovariance) {
    std::string id = "gps.front-1";
    Eigen::Matrix3d input = identity;
    input(0, 1) = 5e-13;
    const auto config = GpsSensorConfig::create(id, input, seed);
    ASSERT_TRUE(config);
    id.clear();
    input.setZero();
    EXPECT_EQ(config->sensorId(), "gps.front-1");
    EXPECT_EQ(config->seed(), seed);
    const auto canonical = config->positionCovarianceWorldM2();
    EXPECT_EQ(canonical(0, 1), 2.5e-13);
    EXPECT_EQ(canonical(1, 0), canonical(0, 1));

    GpsSensor sensor(*config);
    const auto truth = truthAt(Vector3(10.0, -4.0, 1.0));
    ASSERT_TRUE(truth);
    const auto measurement = sensor.measure(*truth, capture);
    ASSERT_TRUE(measurement);
    EXPECT_EQ(measurement->positionCovarianceWorldM2(), canonical);
}

TEST(GpsSensorTest, RejectsInvalidIdsAndCovariances) {
    EXPECT_FALSE(GpsSensorConfig::create("", identity, seed));
    EXPECT_FALSE(GpsSensorConfig::create("gps front", identity, seed));
    EXPECT_FALSE(GpsSensorConfig::create(std::string(65, 'x'), identity, seed));

    Eigen::Matrix3d invalid = identity;
    invalid(0, 0) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(GpsSensorConfig::create("gps", invalid, seed));
    invalid << 1e16, 0.0, 0.0,
                0.0, 1.0, 2.0,
                0.0, 2.0, 1.0;
    EXPECT_FALSE(GpsSensorConfig::create("gps", invalid, seed));
    invalid(1, 2) = 1.5;
    invalid(2, 1) = 0.0;
    EXPECT_FALSE(GpsSensorConfig::create("gps", invalid, seed));
}

TEST(GpsSensorTest, ZeroCovariancePreservesTruthAndAcquisitionThroughInterface) {
    const auto config = GpsSensorConfig::create("gps", Eigen::Matrix3d::Zero(), seed);
    const Vector3 position(-0.0, -5.0, 7.0);
    const auto truth = truthAt(position);
    ASSERT_TRUE(config);
    ASSERT_TRUE(truth);
    GpsSensor sensor(*config);
    SensorModel<GpsMeasurement>& interface = sensor;
    const auto measurement = interface.measure(*truth, capture);
    ASSERT_TRUE(measurement);
    EXPECT_EQ(interface.sensorId(), "gps");
    EXPECT_EQ(measurement->acquisition().sensorId(), "gps");
    EXPECT_EQ(measurement->acquisition().timestamp(), capture);
    EXPECT_EQ(measurement->positionWorldM(), position);
    EXPECT_TRUE(std::signbit(measurement->positionWorldM().x()));
    EXPECT_EQ(measurement->positionCovarianceWorldM2(), Eigen::Matrix3d::Zero());
}

TEST(GpsSensorTest, MatchesIndependentNoiseStreamForCorrelatedAndSingularCovariance) {
    Eigen::Matrix3d correlated;
    correlated << 4.0, -1.0, 0.0,
                 -1.0, 9.0, 2.0,
                  0.0, 2.0, 1.0;
    Eigen::Matrix3d singular;
    singular << 4.0, -6.0, 0.0,
               -6.0, 9.0, 0.0,
                0.0, 0.0, 0.0;
    const Vector3 position(5.0, -6.0, 7.0);
    const auto truth = truthAt(position);
    ASSERT_TRUE(truth);

    for (const auto& covariance : {correlated, singular}) {
        const auto config = GpsSensorConfig::create("gps", covariance, seed);
        const auto noise = GaussianNoise3::create(covariance);
        ASSERT_TRUE(config);
        ASSERT_TRUE(noise);
        GpsSensor sensor(*config);
        GaussianNoise reference(seed);
        for (int i = 0; i < 8; ++i) {
            const auto expected_noise = noise->sample(reference);
            const auto measurement = sensor.measure(*truth, capture);
            ASSERT_TRUE(expected_noise);
            ASSERT_TRUE(measurement);
            EXPECT_EQ(measurement->positionWorldM(), position + *expected_noise);
            EXPECT_NE(measurement->positionWorldM(), position);
            EXPECT_EQ(measurement->positionCovarianceWorldM2(), covariance);
        }
    }
}

TEST(GpsSensorTest, IndependentStreamsReplayAndResetExplicitly) {
    const auto config = GpsSensorConfig::create("gps", identity, seed);
    const auto different = GpsSensorConfig::create("gps", identity, seed + 1);
    const auto truth = truthAt(zero);
    const SimulationTimestamp next_epoch{8, SimulationNanoseconds{0}};
    const auto after_reset = truthAt(zero, next_epoch);
    ASSERT_TRUE(config);
    ASSERT_TRUE(different);
    ASSERT_TRUE(truth);
    ASSERT_TRUE(after_reset);
    GpsSensor first(*config);
    GpsSensor second(*config);
    GpsSensor other(*different);

    const auto initial = first.measure(*truth, capture);
    const auto advanced = first.measure(*after_reset, next_epoch);
    const auto replay_initial = second.measure(*truth, capture);
    const auto replay_advanced = second.measure(*truth, capture);
    const auto other_initial = other.measure(*truth, capture);
    ASSERT_TRUE(initial);
    ASSERT_TRUE(advanced);
    ASSERT_TRUE(replay_initial);
    ASSERT_TRUE(replay_advanced);
    ASSERT_TRUE(other_initial);
    EXPECT_EQ(initial->positionWorldM(), replay_initial->positionWorldM());
    EXPECT_EQ(advanced->positionWorldM(), replay_advanced->positionWorldM());
    EXPECT_NE(initial->positionWorldM(), other_initial->positionWorldM());
    EXPECT_EQ(advanced->acquisition().timestamp(), next_epoch);

    first.reset();
    const auto restarted = first.measure(*after_reset, next_epoch);
    ASSERT_TRUE(restarted);
    EXPECT_EQ(restarted->positionWorldM(), initial->positionWorldM());
    EXPECT_EQ(restarted->acquisition().timestamp(), next_epoch);
}

TEST(GpsSensorTest, CopySnapshotsAdvancedStreamAndAdvancesIndependently) {
    const auto config = GpsSensorConfig::create("gps.copy", identity, seed);
    const auto truth = truthAt(zero);
    const auto noise = GaussianNoise3::create(identity);
    ASSERT_TRUE(config);
    ASSERT_TRUE(truth);
    ASSERT_TRUE(noise);

    GpsSensor original(*config);
    const auto initial = original.measure(*truth, capture);
    ASSERT_TRUE(initial);
    GpsSensor copy(original);

    GaussianNoise reference(seed);
    const auto expected_initial = noise->sample(reference);
    const auto expected_next = noise->sample(reference);
    const auto expected_later = noise->sample(reference);
    ASSERT_TRUE(expected_initial);
    ASSERT_TRUE(expected_next);
    ASSERT_TRUE(expected_later);
    EXPECT_EQ(initial->positionWorldM(), *expected_initial);
    ASSERT_NE(*expected_initial, *expected_next);
    ASSERT_NE(*expected_next, *expected_later);

    // Advance the original twice before either draw from the copy.
    const auto original_next = original.measure(*truth, capture);
    const auto original_later = original.measure(*truth, capture);
    const auto copy_next = copy.measure(*truth, capture);
    const auto copy_later = copy.measure(*truth, capture);
    ASSERT_TRUE(original_next);
    ASSERT_TRUE(original_later);
    ASSERT_TRUE(copy_next);
    ASSERT_TRUE(copy_later);
    EXPECT_EQ(original_next->positionWorldM(), *expected_next);
    EXPECT_EQ(original_later->positionWorldM(), *expected_later);
    EXPECT_EQ(copy_next->positionWorldM(), original_next->positionWorldM());
    EXPECT_EQ(copy_later->positionWorldM(), original_later->positionWorldM());
    for (const auto* measurement : {&*original_next, &*original_later, &*copy_next, &*copy_later}) {
        EXPECT_EQ(measurement->acquisition().timestamp(), capture);
        EXPECT_EQ(measurement->acquisition().sensorId(), "gps.copy");
        EXPECT_EQ(measurement->positionCovarianceWorldM2(), identity);
    }
}

TEST(GpsSensorTest, RejectsMismatchedTimeAndEpochWithoutAdvancingStream) {
    const auto config = GpsSensorConfig::create("gps", identity, seed);
    const auto truth = truthAt(zero);
    ASSERT_TRUE(config);
    ASSERT_TRUE(truth);
    GpsSensor sensor(*config);
    GpsSensor reference(*config);
    EXPECT_FALSE(sensor.measure(*truth, {capture.epoch, SimulationNanoseconds{-1}}));
    EXPECT_FALSE(sensor.measure(*truth, {capture.epoch, capture.elapsed + SimulationNanoseconds{1}}));
    EXPECT_FALSE(sensor.measure(*truth, {capture.epoch + 1, capture.elapsed}));
    const auto actual = sensor.measure(*truth, capture);
    const auto expected = reference.measure(*truth, capture);
    ASSERT_TRUE(actual);
    ASSERT_TRUE(expected);
    EXPECT_EQ(actual->positionWorldM(), expected->positionWorldM());
}

TEST(GpsSensorTest, ValidatedTruthRejectsNonFinitePositionAndExtremeFiniteValuesRemainFinite) {
    const double max = std::numeric_limits<double>::max();
    EXPECT_FALSE(truthAt(Vector3(std::numeric_limits<double>::infinity(), 0.0, 0.0)));
    EXPECT_FALSE(truthAt(Vector3(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)));

    const Eigen::Matrix3d covariance = max * identity;
    const auto config = GpsSensorConfig::create("gps", covariance, seed);
    const Vector3 position(max, -max, max);
    const auto truth = truthAt(position);
    ASSERT_TRUE(config);
    ASSERT_TRUE(truth);
    GpsSensor sensor(*config);
    const auto measurement = sensor.measure(*truth, capture);
    ASSERT_TRUE(measurement);
    EXPECT_TRUE(isFinite(measurement->positionWorldM()));
    EXPECT_EQ(measurement->positionWorldM(), position);
    EXPECT_EQ(measurement->positionCovarianceWorldM2(), covariance);
}

} // namespace
} // namespace aerotwin
