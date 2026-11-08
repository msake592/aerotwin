#include <aerotwin/sensors/ImuSensor.hpp>
#include <aerotwin/simulation/VehicleState.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>

namespace aerotwin {
namespace {

constexpr SimulationTimestamp capture{7, SimulationNanoseconds{123'456}};
constexpr ImuNoiseSeeds seeds{0xfedcba9876543210ULL, 42};
const Vector3 zero = Vector3::Zero();
const Eigen::Matrix3d no_noise = Eigen::Matrix3d::Zero();
const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
const Quaternion aligned = Quaternion::Identity();

static_assert(!std::is_default_constructible_v<ImuSensorConfig>);
static_assert(!std::is_default_constructible_v<ImuKinematics>);

std::optional<VehicleState> truthAt(const Quaternion& orientation = aligned,
                                    const Vector3& rate = zero,
                                    SimulationTimestamp timestamp = capture) {
    return VehicleState::create(zero, orientation, zero, rate, timestamp);
}

TEST(ImuSensorTest, ValidatesConfigurationAndCanonicalizesBothCovariances) {
    Eigen::Matrix3d force_covariance = identity;
    Eigen::Matrix3d rate_covariance = 2.0 * identity;
    force_covariance(0, 1) = 5e-13;
    rate_covariance(2, 0) = -1e-12;
    const Vector3 gravity(0.0, 0.0, -3.0);
    const auto config = ImuSensorConfig::create("imu.main", force_covariance, rate_covariance,
                                               seeds, gravity);
    ASSERT_TRUE(config);
    EXPECT_EQ(config->sensorId(), "imu.main");
    EXPECT_EQ(config->seeds().accelerometer, seeds.accelerometer);
    EXPECT_EQ(config->seeds().gyroscope, seeds.gyroscope);
    EXPECT_EQ(config->gravityWorldMps2(), gravity);

    const auto truth = truthAt();
    const auto kinematics = ImuKinematics::create(gravity, capture);
    ASSERT_TRUE(truth);
    ASSERT_TRUE(kinematics);
    ImuSensor sensor(*config);
    const auto measurement = sensor.measure(*truth, *kinematics, capture);
    ASSERT_TRUE(measurement);
    force_covariance(0, 1) = force_covariance(1, 0) = 2.5e-13;
    rate_covariance(2, 0) = rate_covariance(0, 2) = -5e-13;
    EXPECT_EQ(measurement->specificForceCovarianceBodyM2ps4(), force_covariance);
    EXPECT_EQ(measurement->angularVelocityCovarianceBodyRad2ps2(), rate_covariance);
    EXPECT_EQ(measurement->acquisition().sensorId(), "imu.main");
    EXPECT_EQ(measurement->acquisition().timestamp(), capture);
}

TEST(ImuSensorTest, RejectsInvalidConfigurationAndKinematics) {
    EXPECT_FALSE(ImuSensorConfig::create("", identity, identity, seeds));
    EXPECT_FALSE(ImuSensorConfig::create("imu bad", identity, identity, seeds));
    EXPECT_FALSE(ImuSensorConfig::create("imu", identity, identity, {5, 5}));

    Eigen::Matrix3d invalid = identity;
    invalid(0, 1) = invalid(1, 0) = 2.0;
    EXPECT_FALSE(ImuSensorConfig::create("imu", invalid, identity, seeds));
    EXPECT_FALSE(ImuSensorConfig::create("imu", identity, invalid, seeds));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(ImuSensorConfig::create("imu", identity, identity, seeds, Vector3(0, 0, nan)));
    EXPECT_FALSE(ImuKinematics::create(Vector3(nan, 0, 0), capture));
    EXPECT_FALSE(ImuKinematics::create(Vector3(0, infinity, 0), capture));
    EXPECT_FALSE(ImuKinematics::create(zero, {capture.epoch, SimulationNanoseconds{-1}}));
    EXPECT_FALSE(truthAt(Quaternion(nan, 0, 0, 0)));
    EXPECT_FALSE(truthAt(aligned, Vector3(0, 0, infinity)));
}

TEST(ImuSensorTest, LevelStationaryAndFreeFallWithZeroNoise) {
    const auto config = ImuSensorConfig::create("imu", no_noise, no_noise, seeds);
    const auto truth = truthAt(aligned, Vector3(-0.0, 0.0, 0.0));
    ASSERT_TRUE(config);
    ASSERT_TRUE(truth);
    const auto stationary = ImuKinematics::create(zero, capture);
    const auto falling = ImuKinematics::create(config->gravityWorldMps2(), capture);
    ASSERT_TRUE(stationary);
    ASSERT_TRUE(falling);
    ImuSensor sensor(*config);

    const auto rest = sensor.measure(*truth, *stationary, capture);
    const auto free_fall = sensor.measure(*truth, *falling, capture);
    ASSERT_TRUE(rest);
    ASSERT_TRUE(free_fall);
    EXPECT_EQ(rest->specificForceBodyMps2(), Vector3(0.0, 0.0, 9.80665));
    EXPECT_EQ(free_fall->specificForceBodyMps2(), zero);
    EXPECT_EQ(rest->angularVelocityBodyRadps(), truth->angularVelocityBodyRadps());
    EXPECT_EQ(free_fall->angularVelocityBodyRadps(), truth->angularVelocityBodyRadps());
    EXPECT_TRUE(std::signbit(rest->angularVelocityBodyRadps().x()));
}

TEST(ImuSensorTest, RotatesWorldSpecificForceIntoBodyAndLeavesBodyRateUnrotated) {
    const auto config = ImuSensorConfig::create("imu", no_noise, no_noise, seeds,
                                               Vector3(0.0, 0.0, -4.0));
    const double s = std::sqrt(0.5);
    const Vector3 rate(-1.0, 2.0, 3.0);
    const auto truth = truthAt(Quaternion(s, s, 0.0, 0.0), rate);
    const auto kinematics = ImuKinematics::create(Vector3(1.0, 2.0, 3.0), capture);
    ASSERT_TRUE(config);
    ASSERT_TRUE(truth);
    ASSERT_TRUE(kinematics);
    ImuSensor sensor(*config);

    const auto measurement = sensor.measure(*truth, *kinematics, capture);
    ASSERT_TRUE(measurement);
    // Inverse of +90 degrees about X maps (1, 2, 7) to (1, 7, -2).
    EXPECT_TRUE(measurement->specificForceBodyMps2().isApprox(Vector3(1.0, 7.0, -2.0), 1e-12));
    EXPECT_EQ(measurement->angularVelocityBodyRadps(), rate);
}

TEST(ImuSensorTest, MatchesSeparateReferenceStreamsForCorrelatedAndSingularNoise) {
    Eigen::Matrix3d force_covariance;
    force_covariance << 2.0, -0.5, 0.0,
                       -0.5, 1.0, 0.25,
                        0.0, 0.25, 1.0;
    Eigen::Matrix3d rate_covariance;
    rate_covariance << 4.0, -6.0, 0.0,
                      -6.0, 9.0, 0.0,
                       0.0, 0.0, 0.0;
    const auto config = ImuSensorConfig::create("imu", force_covariance, rate_covariance, seeds, zero);
    const auto force_noise = GaussianNoise3::create(force_covariance);
    const auto rate_noise = GaussianNoise3::create(rate_covariance);
    const Vector3 acceleration(1.0, 2.0, 3.0);
    const Vector3 rate(-1.0, 2.0, -3.0);
    const auto truth = truthAt(aligned, rate);
    const auto kinematics = ImuKinematics::create(acceleration, capture);
    ASSERT_TRUE(config);
    ASSERT_TRUE(force_noise);
    ASSERT_TRUE(rate_noise);
    ASSERT_TRUE(truth);
    ASSERT_TRUE(kinematics);
    ImuSensor sensor(*config);
    GaussianNoise force_stream(seeds.accelerometer);
    GaussianNoise rate_stream(seeds.gyroscope);

    for (int i = 0; i < 8; ++i) {
        const auto expected_force = force_noise->sample(force_stream);
        const auto expected_rate = rate_noise->sample(rate_stream);
        const auto measured = sensor.measure(*truth, *kinematics, capture);
        ASSERT_TRUE(expected_force);
        ASSERT_TRUE(expected_rate);
        ASSERT_TRUE(measured);
        EXPECT_EQ(measured->specificForceBodyMps2(), acceleration + *expected_force);
        EXPECT_EQ(measured->angularVelocityBodyRadps(), rate + *expected_rate);
        EXPECT_EQ(measured->specificForceCovarianceBodyM2ps4(), force_covariance);
        EXPECT_EQ(measured->angularVelocityCovarianceBodyRad2ps2(), rate_covariance);
    }
}

TEST(ImuSensorTest, StreamsReplayCopyAndResetWithoutAutomaticEpochReset) {
    const auto config = ImuSensorConfig::create("imu", identity, identity, seeds, zero);
    const auto other_config = ImuSensorConfig::create(
        "imu", identity, identity, {seeds.accelerometer + 1, seeds.gyroscope}, zero);
    constexpr SimulationTimestamp next_epoch{capture.epoch + 1, SimulationNanoseconds{0}};
    const auto truth = truthAt();
    const auto next_truth = truthAt(aligned, zero, next_epoch);
    const auto kinematics = ImuKinematics::create(zero, capture);
    const auto next_kinematics = ImuKinematics::create(zero, next_epoch);
    ASSERT_TRUE(config);
    ASSERT_TRUE(other_config);
    ASSERT_TRUE(truth);
    ASSERT_TRUE(next_truth);
    ASSERT_TRUE(kinematics);
    ASSERT_TRUE(next_kinematics);
    ImuSensor sensor(*config);
    ImuSensor reference(*config);
    ImuSensor other(*other_config);

    const auto first = sensor.measure(*truth, *kinematics, capture);
    const auto replay = reference.measure(*truth, *kinematics, capture);
    const auto different = other.measure(*truth, *kinematics, capture);
    ASSERT_TRUE(first);
    ASSERT_TRUE(replay);
    ASSERT_TRUE(different);
    EXPECT_EQ(first->specificForceBodyMps2(), replay->specificForceBodyMps2());
    EXPECT_EQ(first->angularVelocityBodyRadps(), replay->angularVelocityBodyRadps());
    EXPECT_NE(first->specificForceBodyMps2(), different->specificForceBodyMps2());
    EXPECT_EQ(first->angularVelocityBodyRadps(), different->angularVelocityBodyRadps());

    ImuSensor copy(sensor);
    const auto advanced = sensor.measure(*next_truth, *next_kinematics, next_epoch);
    const auto later = sensor.measure(*next_truth, *next_kinematics, next_epoch);
    const auto copied = copy.measure(*next_truth, *next_kinematics, next_epoch);
    const auto copied_later = copy.measure(*next_truth, *next_kinematics, next_epoch);
    const auto expected = reference.measure(*truth, *kinematics, capture);
    ASSERT_TRUE(advanced);
    ASSERT_TRUE(later);
    ASSERT_TRUE(copied);
    ASSERT_TRUE(copied_later);
    ASSERT_TRUE(expected);
    EXPECT_NE(advanced->specificForceBodyMps2(), first->specificForceBodyMps2());
    EXPECT_NE(advanced->angularVelocityBodyRadps(), first->angularVelocityBodyRadps());
    EXPECT_EQ(advanced->specificForceBodyMps2(), expected->specificForceBodyMps2());
    EXPECT_EQ(advanced->angularVelocityBodyRadps(), expected->angularVelocityBodyRadps());
    EXPECT_EQ(copied->specificForceBodyMps2(), advanced->specificForceBodyMps2());
    EXPECT_EQ(copied->angularVelocityBodyRadps(), advanced->angularVelocityBodyRadps());
    EXPECT_EQ(copied_later->specificForceBodyMps2(), later->specificForceBodyMps2());
    EXPECT_EQ(copied_later->angularVelocityBodyRadps(), later->angularVelocityBodyRadps());
    EXPECT_EQ(advanced->acquisition().timestamp(), next_epoch);

    sensor.reset();
    const auto restarted = sensor.measure(*next_truth, *next_kinematics, next_epoch);
    ASSERT_TRUE(restarted);
    EXPECT_EQ(restarted->specificForceBodyMps2(), first->specificForceBodyMps2());
    EXPECT_EQ(restarted->angularVelocityBodyRadps(), first->angularVelocityBodyRadps());
}

TEST(ImuSensorTest, RejectsMismatchesAndSubtractionOverflowWithoutDrawing) {
    const double max = std::numeric_limits<double>::max();
    const Vector3 gravity(max, 0.0, 0.0);
    const auto config = ImuSensorConfig::create("imu", identity, identity, seeds, gravity);
    const auto truth = truthAt();
    const auto valid = ImuKinematics::create(gravity, capture);
    const auto overflows = ImuKinematics::create(-gravity, capture);
    const SimulationTimestamp other_time{capture.epoch, capture.elapsed + SimulationNanoseconds{1}};
    const auto wrong_time = ImuKinematics::create(gravity, other_time);
    const auto wrong_epoch = ImuKinematics::create(gravity, {capture.epoch + 1, capture.elapsed});
    ASSERT_TRUE(config);
    ASSERT_TRUE(truth);
    ASSERT_TRUE(valid);
    ASSERT_TRUE(overflows);
    ASSERT_TRUE(wrong_time);
    ASSERT_TRUE(wrong_epoch);
    ImuSensor sensor(*config);
    ImuSensor reference(*config);

    EXPECT_FALSE(sensor.measure(*truth, *valid, {capture.epoch, SimulationNanoseconds{-1}}));
    EXPECT_FALSE(sensor.measure(*truth, *wrong_time, other_time));
    EXPECT_FALSE(sensor.measure(*truth, *wrong_time, capture));
    EXPECT_FALSE(sensor.measure(*truth, *wrong_epoch, capture));
    EXPECT_FALSE(sensor.measure(*truth, *overflows, capture));
    const auto measured = sensor.measure(*truth, *valid, capture);
    const auto expected = reference.measure(*truth, *valid, capture);
    ASSERT_TRUE(measured);
    ASSERT_TRUE(expected);
    EXPECT_EQ(measured->specificForceBodyMps2(), expected->specificForceBodyMps2());
    EXPECT_EQ(measured->angularVelocityBodyRadps(), expected->angularVelocityBodyRadps());
}

TEST(ImuSensorTest, RotationOverflowDoesNotAdvanceEitherNoiseStream) {
    const Eigen::Matrix3d rate_covariance = 2.0 * identity;
    const auto config = ImuSensorConfig::create("imu.overflow", identity, rate_covariance,
                                               seeds, zero);
    const auto truth = truthAt();
    const auto kinematics = ImuKinematics::create(zero, capture);
    ASSERT_TRUE(config);
    ASSERT_TRUE(truth);
    ASSERT_TRUE(kinematics);
    ImuSensor sensor(*config);
    ImuSensor reference(*config);

    const auto first = sensor.measure(*truth, *kinematics, capture);
    const auto reference_first = reference.measure(*truth, *kinematics, capture);
    ASSERT_TRUE(first);
    ASSERT_TRUE(reference_first);
    EXPECT_EQ(first->specificForceBodyMps2(), reference_first->specificForceBodyMps2());
    EXPECT_EQ(first->angularVelocityBodyRadps(), reference_first->angularVelocityBodyRadps());

    const double max = std::numeric_limits<double>::max();
    const double half_angle = std::numbers::pi / 8.0;
    const auto tilted = truthAt(Quaternion(std::cos(half_angle), 0.0, 0.0, std::sin(half_angle)));
    const auto extreme = ImuKinematics::create(Vector3(max, max, 0.0), capture);
    ASSERT_TRUE(tilted);
    ASSERT_TRUE(extreme);
    // Zero gravity leaves the subtraction finite. The inverse 45-degree
    // rotation then requires a body X component of sqrt(2) times max.
    ASSERT_FALSE(sensor.measure(*tilted, *extreme, capture));

    const auto measured = sensor.measure(*truth, *kinematics, capture);
    const auto expected = reference.measure(*truth, *kinematics, capture);
    ASSERT_TRUE(measured);
    ASSERT_TRUE(expected);
    EXPECT_NE(first->specificForceBodyMps2(), expected->specificForceBodyMps2());
    EXPECT_NE(first->angularVelocityBodyRadps(), expected->angularVelocityBodyRadps());
    EXPECT_EQ(measured->specificForceBodyMps2(), expected->specificForceBodyMps2());
    EXPECT_EQ(measured->angularVelocityBodyRadps(), expected->angularVelocityBodyRadps());
    for (const auto* sample : {&*measured, &*expected}) {
        EXPECT_EQ(sample->acquisition().sensorId(), "imu.overflow");
        EXPECT_EQ(sample->acquisition().timestamp(), capture);
        EXPECT_EQ(sample->specificForceCovarianceBodyM2ps4(), identity);
        EXPECT_EQ(sample->angularVelocityCovarianceBodyRad2ps2(), rate_covariance);
    }
}

TEST(ImuSensorTest, RejectsUnrepresentableRotationAndAcceptsExtremeAlignedValues) {
    const double max = std::numeric_limits<double>::max();
    const auto config = ImuSensorConfig::create("imu", no_noise, no_noise, seeds, zero);
    const double half_angle = std::numbers::pi / 8.0;
    const auto tilted = truthAt(Quaternion(std::cos(half_angle), 0.0, 0.0, std::sin(half_angle)));
    const auto aligned_truth = truthAt(aligned, Vector3(max, -max, max));
    const auto kinematics = ImuKinematics::create(Vector3(max, max, 0.0), capture);
    ASSERT_TRUE(config);
    ASSERT_TRUE(tilted);
    ASSERT_TRUE(aligned_truth);
    ASSERT_TRUE(kinematics);
    ImuSensor sensor(*config);

    EXPECT_FALSE(sensor.measure(*tilted, *kinematics, capture));
    const auto valid = sensor.measure(*aligned_truth, *kinematics, capture);
    ASSERT_TRUE(valid);
    EXPECT_EQ(valid->specificForceBodyMps2(), kinematics->accelerationWorldMps2());
    EXPECT_EQ(valid->angularVelocityBodyRadps(), aligned_truth->angularVelocityBodyRadps());
}

} // namespace
} // namespace aerotwin
