#include <aerotwin/sensors/SensorModel.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <type_traits>

namespace aerotwin {
namespace {

static_assert(std::is_abstract_v<SensorModel<GpsMeasurement>>);
static_assert(std::has_virtual_destructor_v<SensorModel<ImuMeasurement>>);
static_assert(!std::is_default_constructible_v<GpsMeasurement>);
static_assert(!std::is_default_constructible_v<ImuMeasurement>);

constexpr SimulationTimestamp capture{9, SimulationNanoseconds{123'456}};
const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
const Vector3 zero = Vector3::Zero();

void expectCovariance(const Eigen::Matrix3d& covariance, bool accepted) {
    const auto acquisition = MeasurementAcquisition::create("sensor", capture);
    ASSERT_TRUE(acquisition);
    EXPECT_EQ(GpsMeasurement::create(*acquisition, zero, covariance).has_value(), accepted);
    EXPECT_EQ(ImuMeasurement::create(*acquisition, zero, zero, covariance, identity).has_value(),
              accepted);
    EXPECT_EQ(ImuMeasurement::create(*acquisition, zero, zero, identity, covariance).has_value(),
              accepted);
}

TEST(MeasurementsTest, GpsPreservesEnuAxesAndAcquisition) {
    std::string sensor = "gps.front-1";
    const auto acquisition = MeasurementAcquisition::create(sensor, capture);
    ASSERT_TRUE(acquisition);
    sensor.clear();
    const Vector3 position(12.0, -4.0, 2.0);
    const Eigen::Matrix3d covariance = Vector3(4.0, 9.0, 16.0).asDiagonal();
    const auto gps = GpsMeasurement::create(*acquisition, position, covariance);
    ASSERT_TRUE(gps);
    EXPECT_EQ(gps->positionWorldM(), position);
    EXPECT_EQ(gps->positionCovarianceWorldM2(), covariance);
    EXPECT_EQ(gps->acquisition().sensorId(), "gps.front-1");
    EXPECT_EQ(gps->acquisition().timestamp(), capture);
}

TEST(MeasurementsTest, ImuPreservesFluSpecificForceRateAndSeparateCovariances) {
    const auto acquisition = MeasurementAcquisition::create("imu", capture);
    ASSERT_TRUE(acquisition);
    const Vector3 force(1.0, -2.0, 9.81);
    const Vector3 rate(-0.4, 0.5, 0.6);
    const Eigen::Matrix3d force_covariance = 2.0 * identity;
    const Eigen::Matrix3d rate_covariance = 0.125 * identity;
    const auto imu = ImuMeasurement::create(*acquisition, force, rate, force_covariance,
                                            rate_covariance);
    ASSERT_TRUE(imu);
    EXPECT_EQ(imu->specificForceBodyMps2(), force);
    EXPECT_EQ(imu->angularVelocityBodyRadps(), rate);
    EXPECT_EQ(imu->specificForceCovarianceBodyM2ps4(), force_covariance);
    EXPECT_EQ(imu->angularVelocityCovarianceBodyRad2ps2(), rate_covariance);
    EXPECT_EQ(imu->acquisition().sensorId(), "imu");
    EXPECT_EQ(imu->acquisition().timestamp(), capture);
}

TEST(MeasurementsTest, RejectsInvalidIdentifiersAndNegativeElapsedTime) {
    EXPECT_FALSE(MeasurementAcquisition::create("", capture));
    EXPECT_FALSE(MeasurementAcquisition::create("imu 1", capture));
    EXPECT_FALSE(MeasurementAcquisition::create("imu\n1", capture));
    EXPECT_FALSE(MeasurementAcquisition::create(std::string(65, 'x'), capture));
    EXPECT_TRUE(MeasurementAcquisition::create(std::string(64, 'x'), capture));
    EXPECT_FALSE(MeasurementAcquisition::create("gps", {9, SimulationNanoseconds{-1}}));
}

TEST(MeasurementsTest, RejectsNonFiniteObservationsAndCovarianceEntries) {
    const auto acquisition = MeasurementAcquisition::create("sensor", capture);
    ASSERT_TRUE(acquisition);
    const Vector3 invalid(0.0, std::numeric_limits<double>::infinity(), 0.0);
    EXPECT_FALSE(GpsMeasurement::create(*acquisition, invalid, identity));
    EXPECT_FALSE(ImuMeasurement::create(*acquisition, invalid, zero, identity, identity));
    EXPECT_FALSE(ImuMeasurement::create(*acquisition, zero, invalid, identity, identity));

    Eigen::Matrix3d invalid_covariance = identity;
    invalid_covariance(0, 1) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(GpsMeasurement::create(*acquisition, zero, invalid_covariance));
    EXPECT_FALSE(ImuMeasurement::create(*acquisition, zero, zero, invalid_covariance, identity));
    EXPECT_FALSE(ImuMeasurement::create(*acquisition, zero, zero, identity, invalid_covariance));
}

TEST(MeasurementsTest, SymmetrizesRoundoffAndRejectsMaterialAsymmetry) {
    const auto acquisition = MeasurementAcquisition::create("gps", capture);
    ASSERT_TRUE(acquisition);
    Eigen::Matrix3d covariance = identity;
    covariance(0, 1) = 5e-13;
    const auto gps = GpsMeasurement::create(*acquisition, zero, covariance);
    ASSERT_TRUE(gps);
    const auto stored = gps->positionCovarianceWorldM2();
    EXPECT_EQ(stored(0, 1), 2.5e-13);
    EXPECT_EQ(stored(1, 0), stored(0, 1));
    EXPECT_EQ(covariance(1, 0), 0.0);
    const auto imu = ImuMeasurement::create(*acquisition, zero, zero, covariance, covariance);
    ASSERT_TRUE(imu);
    EXPECT_EQ(imu->specificForceCovarianceBodyM2ps4(), stored);
    EXPECT_EQ(imu->angularVelocityCovarianceBodyRad2ps2(), stored);

    covariance(0, 1) = 2e-12;
    EXPECT_FALSE(GpsMeasurement::create(*acquisition, zero, covariance));
}

TEST(MeasurementsTest, AcceptsSingularPsdAndRejectsNegativeVarianceAndIndefiniteMatrices) {
    expectCovariance(Eigen::Matrix3d::Zero(), true);
    expectCovariance(Eigen::Matrix3d::Ones(), true);

    Eigen::Matrix3d covariance = identity;
    covariance(0, 0) = -1e-15;
    expectCovariance(covariance, false);

    covariance = identity;
    covariance(0, 1) = covariance(1, 0) = 1.0 + 1e-13;
    expectCovariance(covariance, true);
    covariance(0, 1) = covariance(1, 0) = 1.01;
    expectCovariance(covariance, false);
}

TEST(MeasurementsTest, RejectsIndefiniteAndAsymmetricMixedScaleCovariances) {
    Eigen::Matrix3d covariance;
    covariance << 1e16, 0.0, 0.0,
                  0.0, 1.0, 2.0,
                  0.0, 2.0, 1.0;
    expectCovariance(covariance, false);

    covariance(1, 2) = 1.5;
    covariance(2, 1) = 0.0;
    expectCovariance(covariance, false);

    // Each pair satisfies the correlation bound, but the full matrix is indefinite.
    covariance << 1e16, 9e7, 9e7,
                  9e7, 1.0, -0.9,
                  9e7, -0.9, 1.0;
    expectCovariance(covariance, false);
}

TEST(MeasurementsTest, AcceptsValidMixedScaleCovariance) {
    Eigen::Matrix3d covariance;
    covariance << 1e300, 0.5, 0.0,
                  0.5, 1e-300, 0.0,
                  0.0, 0.0, 1.0;
    expectCovariance(covariance, true);
}

TEST(MeasurementsTest, RequiresExactZeroCovariancesOnZeroVarianceAxes) {
    Eigen::Matrix3d covariance = Vector3(1e16, 1.0, 0.0).asDiagonal();
    expectCovariance(covariance, true);
    covariance(2, 1) = std::numeric_limits<double>::denorm_min();
    expectCovariance(covariance, false);
    covariance(1, 2) = covariance(2, 1);
    expectCovariance(covariance, false);
}

TEST(MeasurementsTest, CovariancePolicyHandlesExtremeFiniteScales) {
    const double tiny = std::numeric_limits<double>::denorm_min();
    const double huge = std::numeric_limits<double>::max();
    expectCovariance(Eigen::Matrix3d::Constant(tiny), true);
    expectCovariance(Eigen::Matrix3d::Constant(huge), true);

    // A geometric mean rounded to subnormal double before division would
    // incorrectly turn the first correlation into 1 instead of 1/sqrt(2).
    Eigen::Matrix3d covariance;
    covariance << tiny, tiny, tiny,
                  tiny, 2.0 * tiny, 0.0,
                  tiny, 0.0, 2.0 * tiny;
    expectCovariance(covariance, true);

    covariance = Eigen::Matrix3d::Constant(huge);
    covariance.diagonal().setConstant(tiny);
    expectCovariance(covariance, false);
}

} // namespace
} // namespace aerotwin
