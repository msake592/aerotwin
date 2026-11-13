#include <aerotwin/estimation/GpsKalmanUpdater.hpp>
#include <aerotwin/estimation/KalmanPredictor.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace aerotwin {
namespace {

using Covariance = EstimatorCovariance6;
using State = EstimatorVector6;
using Matrix = Eigen::Matrix3d;
using Time = SimulationNanoseconds;

EstimatorState estimate(const Covariance& covariance, const State& state = State::Zero(),
                        SimulationTimestamp timestamp = {}) {
    return std::get<EstimatorState>(EstimatorState::create(state, covariance, timestamp));
}

GpsMeasurement gps(const Matrix& covariance, const Vector3& position = Vector3::Zero(),
                   SimulationTimestamp timestamp = {}) {
    return GpsMeasurement::create(MeasurementAcquisition::create("gps", timestamp).value(),
                                  position, covariance).value();
}

void expectError(const GpsUpdateResult& result, GpsUpdateError expected) {
    ASSERT_TRUE(std::holds_alternative<GpsUpdateError>(result));
    EXPECT_EQ(std::get<GpsUpdateError>(result), expected);
}

TEST(GpsKalmanUpdaterTest, IntegratesPredictionAndCorrectsPositionAndVelocityOnAllAxes) {
    State state;
    state << 1, 2, 3, 4, 5, 6;
    const auto prediction = KalmanPredictor::predict(estimate(Covariance::Identity(), state),
                                                     {0, Time{1'000'000'000}}, Matrix::Zero());
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(prediction));
    const auto& prior = std::get<EstimatorState>(prediction);
    const auto measurement = gps(2 * Matrix::Identity(), Vector3(7, 3, 15), prior.timestamp());
    const auto result = GpsKalmanUpdater::update(prior, measurement);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& output = std::get<EstimatorState>(result);
    // Each independent axis has Ppp=2, Pvp=1, S=4 and gains 1/2 and 1/4.
    EXPECT_TRUE(output.positionWorldM().isApprox(Vector3(6, 5, 12), 1e-14));
    EXPECT_TRUE(output.velocityWorldMps().isApprox(Vector3(4.5, 4, 7.5), 1e-14));
    Covariance expected = Covariance::Identity();
    expected.bottomRightCorner<3, 3>() *= 0.75;
    expected.topRightCorner<3, 3>() = expected.bottomLeftCorner<3, 3>() = 0.5 * Matrix::Identity();
    EXPECT_TRUE(output.covariance().isApprox(expected, 1e-14));
    EXPECT_EQ(output.timestamp(), prior.timestamp());
    EXPECT_EQ(prior.positionWorldM(), Vector3(5, 7, 9));
    EXPECT_EQ(prior.velocityWorldMps(), Vector3(4, 5, 6));
    EXPECT_EQ(prior.covariance()(0, 0), 2);
    EXPECT_EQ(measurement.positionWorldM(), Vector3(7, 3, 15));
    const auto replay = GpsKalmanUpdater::update(prior, measurement);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(replay));
    EXPECT_EQ(std::get<EstimatorState>(replay).vector(), output.vector());
    EXPECT_EQ(std::get<EstimatorState>(replay).covariance(), output.covariance());
}

TEST(GpsKalmanUpdaterTest, SolvesCorrelatedPositionModesWithSingularPrior) {
    Covariance prior = Covariance::Zero();
    prior.topLeftCorner<3, 3>() << 3, 1, 0, 1, 3, 0, 0, 0, 0;
    Matrix noise;
    noise << 3, -1, 0, -1, 3, 0, 0, 0, 1;
    const auto result = GpsKalmanUpdater::update(estimate(prior), gps(noise, Vector3(6, 0, 9)));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& output = std::get<EstimatorState>(result);
    EXPECT_TRUE(output.positionWorldM().isApprox(Vector3(3, 1, 0), 1e-14));
    Covariance expected = Covariance::Zero();
    expected(0, 0) = expected(1, 1) = 4.0 / 3.0;
    EXPECT_TRUE(output.covariance().isApprox(expected, 1e-14));
}

TEST(GpsKalmanUpdaterTest, HandlesZeroInnovationAndZeroMeasurementCovariance) {
    const auto prior = estimate(Covariance::Identity(), State::Ones());
    const auto result = GpsKalmanUpdater::update(prior, gps(Matrix::Zero(), Vector3::Ones()));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    EXPECT_EQ(std::get<EstimatorState>(result).vector(), prior.vector());
    Covariance expected = Covariance::Identity();
    expected.topLeftCorner<3, 3>().setZero();
    EXPECT_EQ(std::get<EstimatorState>(result).covariance(), expected);

    State distant = State::Ones();
    distant(0) = 1e30;
    const auto exact = GpsKalmanUpdater::update(estimate(Covariance::Identity(), distant),
                                               gps(Matrix::Zero(), Vector3::Ones()));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(exact));
    EXPECT_EQ(std::get<EstimatorState>(exact).positionWorldM(), Vector3::Ones());
}

TEST(GpsKalmanUpdaterTest, KeepsZeroPriorCovarianceAndState) {
    const auto prior = estimate(Covariance::Zero(), State::Ones());
    const auto result = GpsKalmanUpdater::update(prior, gps(Matrix::Identity()));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    EXPECT_EQ(std::get<EstimatorState>(result).vector(), prior.vector());
    EXPECT_EQ(std::get<EstimatorState>(result).covariance(), Covariance::Zero());
}

TEST(GpsKalmanUpdaterTest, RejectsSingularAndNumericallyRankDeficientInnovation) {
    const auto prior = estimate(Covariance::Zero());
    expectError(GpsKalmanUpdater::update(prior, gps(Matrix::Zero())),
                GpsUpdateError::UnsolvableInnovationCovariance);
    Matrix noise = Matrix::Identity();
    noise(0, 1) = noise(1, 0) = 1.0;
    expectError(GpsKalmanUpdater::update(prior, gps(noise)),
                GpsUpdateError::UnsolvableInnovationCovariance);
    noise(0, 1) = noise(1, 0) = std::nextafter(1.0, 0.0);
    expectError(GpsKalmanUpdater::update(prior, gps(noise)),
                GpsUpdateError::UnsolvableInnovationCovariance);
}

TEST(GpsKalmanUpdaterTest, SolvesWidelyDifferentPositiveVariances) {
    Matrix noise = Matrix::Zero();
    noise.diagonal() << 1e-300, 1, 1e300;
    Covariance covariance = Covariance::Zero();
    covariance.topLeftCorner<3, 3>() = noise;
    const auto result = GpsKalmanUpdater::update(estimate(covariance), gps(noise, Vector3::Ones()));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& output = std::get<EstimatorState>(result);
    EXPECT_TRUE(output.positionWorldM().isApprox(Vector3::Constant(0.5), 1e-14));
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR(output.covariance()(axis, axis) / noise(axis, axis), 0.5, 1e-14);
    }
}

TEST(GpsKalmanUpdaterTest, PreservesRepresentableSubnormalJosephSum) {
    const double s = std::numeric_limits<double>::denorm_min();
    const auto result = GpsKalmanUpdater::update(estimate(2 * s * Covariance::Identity()),
                                                 gps(2 * s * Matrix::Identity()));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    Covariance expected = 2 * s * Covariance::Identity();
    expected.topLeftCorner<3, 3>() = s * Matrix::Identity();
    EXPECT_EQ(std::get<EstimatorState>(result).covariance(), expected);
}

TEST(GpsKalmanUpdaterTest, AppliesGainBelowDoubleRangeToLargeInnovation) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = 1e-100;
    Matrix noise = Matrix::Identity();
    noise(0, 0) = 1e300;
    const SimulationTimestamp timestamp{7, Time{123}};
    const Vector3 observation(1e300, 0, 0);
    const auto prior = estimate(covariance, State::Zero(), timestamp);
    const auto measurement = gps(noise, observation, timestamp);
    const auto result = GpsKalmanUpdater::update(prior, measurement);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& output = std::get<EstimatorState>(result);
    EXPECT_NEAR(output.vector()(0) / 1e-100, 1.0, 1e-14);
    EXPECT_TRUE(output.vector().tail<5>().isZero(0.0));
    EXPECT_EQ(output.covariance()(0, 0), covariance(0, 0));
    EXPECT_EQ(output.timestamp(), timestamp);
    EXPECT_EQ(prior.vector(), State::Zero());
    EXPECT_EQ(prior.covariance(), covariance);
    EXPECT_EQ(prior.timestamp(), timestamp);
    EXPECT_EQ(measurement.positionWorldM(), observation);
    EXPECT_EQ(measurement.positionCovarianceWorldM2(), noise);
    EXPECT_EQ(measurement.acquisition().timestamp(), timestamp);
}

TEST(GpsKalmanUpdaterTest, PreservesSubnormalJosephSumAcrossMixedScales) {
    const double s = std::numeric_limits<double>::denorm_min();
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = 2 * s;
    Matrix noise = Matrix::Identity();
    noise(0, 0) = 2 * s;
    const SimulationTimestamp timestamp{9, Time{456}};
    const auto prior = estimate(covariance, State::Zero(), timestamp);
    const auto measurement = gps(noise, Vector3::Zero(), timestamp);
    const auto result = GpsKalmanUpdater::update(prior, measurement);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& output = std::get<EstimatorState>(result);
    Covariance expected = Covariance::Identity();
    expected(0, 0) = s;
    expected(1, 1) = expected(2, 2) = 0.5;
    EXPECT_EQ(output.covariance()(0, 0), s);
    EXPECT_TRUE(output.covariance().isApprox(expected, 1e-14));
    EXPECT_EQ(output.timestamp(), timestamp);
    EXPECT_EQ(prior.vector(), State::Zero());
    EXPECT_EQ(prior.covariance(), covariance);
    EXPECT_EQ(prior.timestamp(), timestamp);
    EXPECT_EQ(measurement.positionWorldM(), Vector3::Zero());
    EXPECT_EQ(measurement.positionCovarianceWorldM2(), noise);
    EXPECT_EQ(measurement.acquisition().timestamp(), timestamp);
}

TEST(GpsKalmanUpdaterTest, PreservesPositionWeightsWhenInnovationLosesObservation) {
    State state = State::Zero();
    state(0) = 1e30;
    const Covariance covariance = Covariance::Identity();
    const Matrix noise = 1e-30 * Matrix::Identity();
    const Vector3 observation(1, 0, 0);
    const SimulationTimestamp timestamp{7, Time{123}};
    const auto prior = estimate(covariance, state, timestamp);
    const auto measurement = gps(noise, observation, timestamp);
    const auto result = GpsKalmanUpdater::update(prior, measurement);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& output = std::get<EstimatorState>(result);
    EXPECT_NEAR(output.vector()(0), 2.0, 1e-14);
    EXPECT_TRUE(output.vector().tail<5>().isZero(0.0));
    EXPECT_NEAR(output.covariance()(0, 0) / 1e-30, 1.0, 1e-14);
    EXPECT_EQ(output.timestamp(), timestamp);
    EXPECT_EQ(prior.vector(), state);
    EXPECT_EQ(prior.covariance(), covariance);
    EXPECT_EQ(prior.timestamp(), timestamp);
    EXPECT_EQ(measurement.positionWorldM(), observation);
    EXPECT_EQ(measurement.positionCovarianceWorldM2(), noise);
    EXPECT_EQ(measurement.acquisition().timestamp(), timestamp);
}

TEST(GpsKalmanUpdaterTest, PreservesVelocityObservationAfterPriorCancellation) {
    State state = State::Zero();
    state(0) = state(3) = 1e30;
    Covariance covariance = Covariance::Identity();
    covariance(0, 3) = covariance(3, 0) = 1.0;
    const auto result = GpsKalmanUpdater::update(estimate(covariance, state),
                                                gps(Matrix::Zero(), Vector3(1, 0, 0)));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    EXPECT_EQ(std::get<EstimatorState>(result).velocityWorldMps(), Vector3(1, 0, 0));
}

TEST(GpsKalmanUpdaterTest, PreservesVelocityCorrectionOnChangedAxis) {
    State state = State::Zero();
    state(0) = 1e30;
    Covariance covariance = Covariance::Identity();
    covariance(3, 3) = 3;
    covariance(3, 0) = covariance(0, 3) = 1;
    covariance(3, 1) = covariance(1, 3) = 1;
    const SimulationTimestamp timestamp{7, Time{123}};
    const Vector3 observation(1e30, 1, 0);
    const auto prior = estimate(covariance, state, timestamp);
    const auto measurement = gps(Matrix::Zero(), observation, timestamp);
    const auto result = GpsKalmanUpdater::update(prior, measurement);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& output = std::get<EstimatorState>(result);
    EXPECT_EQ(output.positionWorldM(), observation);
    EXPECT_EQ(output.velocityWorldMps(), Vector3(1, 0, 0));
    Covariance expected = Covariance::Zero();
    expected.bottomRightCorner<3, 3>().setIdentity();
    EXPECT_EQ(output.covariance(), expected);
    EXPECT_EQ(output.timestamp(), timestamp);
    EXPECT_EQ(prior.vector(), state);
    EXPECT_EQ(prior.covariance(), covariance);
    EXPECT_EQ(prior.timestamp(), timestamp);
    EXPECT_EQ(measurement.positionWorldM(), observation);
    EXPECT_EQ(measurement.positionCovarianceWorldM2(), Matrix::Zero());
    EXPECT_EQ(measurement.acquisition().timestamp(), timestamp);
}

TEST(GpsKalmanUpdaterTest, RequiresExactEpochAndElapsedTime) {
    const auto prior = estimate(Covariance::Identity(), State::Zero(), {7, Time::max() - Time{1}});
    const auto time = prior.timestamp().elapsed;
    expectError(GpsKalmanUpdater::update(prior, gps(Matrix::Identity(), Vector3::Zero(), {8, time})),
                GpsUpdateError::EpochMismatch);
    for (const auto elapsed : {time - Time{1}, time + Time{1}}) {
        expectError(GpsKalmanUpdater::update(prior, gps(Matrix::Identity(), Vector3::Zero(),
                                                       {7, elapsed})),
                    GpsUpdateError::TimestampMismatch);
    }
    const auto result = GpsKalmanUpdater::update(prior, gps(Matrix::Identity(), Vector3::Zero(),
                                                           {7, time}));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    EXPECT_EQ(std::get<EstimatorState>(result).timestamp(), prior.timestamp());
}

TEST(GpsKalmanUpdaterTest, RejectsNonfiniteInnovationAndInnovationCovariance) {
    const double largest = std::numeric_limits<double>::max();
    State state = State::Zero();
    state(0) = -largest;
    expectError(GpsKalmanUpdater::update(estimate(Covariance::Identity(), state),
                                        gps(Matrix::Identity(), Vector3(largest, 0, 0))),
                GpsUpdateError::NumericalFailure);
    expectError(GpsKalmanUpdater::update(estimate(largest * Covariance::Identity()),
                                        gps(largest * Matrix::Identity())),
                GpsUpdateError::NumericalFailure);
}

TEST(GpsKalmanUpdaterTest, RejectsInvalidPosteriorFromToleranceValidPrior) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 3) = covariance(3, 0) = 1.0 + 2e-13;
    const auto prior = estimate(covariance);
    expectError(GpsKalmanUpdater::update(prior, gps(Matrix::Zero())),
                GpsUpdateError::InvalidPosteriorCovariance);
    EXPECT_EQ(prior.covariance(), covariance);
}

} // namespace
} // namespace aerotwin
