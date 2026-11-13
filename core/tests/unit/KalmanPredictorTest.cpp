#include <aerotwin/estimation/KalmanPredictor.hpp>

#include <gtest/gtest.h>

#include <limits>

namespace aerotwin {
namespace {

using Covariance = EstimatorCovariance6;
using StateVector = EstimatorVector6;
using Density = Eigen::Matrix3d;
using Time = SimulationNanoseconds;

EstimatorState initial(const StateVector& state = StateVector::Zero(),
                       const Covariance& covariance = Covariance::Zero(),
                       SimulationTimestamp timestamp = {}) {
    return std::get<EstimatorState>(EstimatorState::create(state, covariance, timestamp));
}

void expectError(const PredictionResult& result, PredictionError error) {
    ASSERT_TRUE(std::holds_alternative<PredictionError>(result));
    EXPECT_EQ(std::get<PredictionError>(result), error);
}

TEST(KalmanPredictorTest, AdvancesAllPositionAxesAndPreservesVelocityAndInput) {
    StateVector vector;
    vector << 1, 2, 3, 4, -5, 6;
    const auto start = initial(vector, Covariance::Zero(), {7, Time{100}});
    const SimulationTimestamp target{7, Time{2'000'000'100}};
    const auto result = KalmanPredictor::predict(start, target, Density::Zero());
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& estimate = std::get<EstimatorState>(result);
    EXPECT_EQ(estimate.positionWorldM(), Vector3(9, -8, 15));
    EXPECT_EQ(estimate.velocityWorldMps(), Vector3(4, -5, 6));
    EXPECT_EQ(estimate.covariance(), Covariance::Zero());
    EXPECT_EQ(estimate.timestamp(), target);
    EXPECT_EQ(start.vector(), vector);
    EXPECT_EQ(start.covariance(), Covariance::Zero());
    EXPECT_EQ(start.timestamp(), (SimulationTimestamp{7, Time{100}}));
}

TEST(KalmanPredictorTest, PropagatesCorrelatedSingularCovarianceWithSparseTransition) {
    StateVector direction;
    direction << 1, -2, 3, -4, 5, 6;
    const Covariance covariance = direction * direction.transpose();
    const auto start = initial(StateVector::Zero(), covariance);
    const auto result = KalmanPredictor::predict(start, {0, Time{2'000'000'000}}, Density::Zero());
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    StateVector transitioned;
    transitioned << -7, 8, 15, -4, 5, 6;
    const auto& estimate = std::get<EstimatorState>(result);
    EXPECT_EQ(estimate.vector(), StateVector::Zero());
    EXPECT_EQ(estimate.covariance(), (transitioned * transitioned.transpose()).eval());
}

TEST(KalmanPredictorTest, IntegratesCorrelatedSingularWhiteAccelerationNoise) {
    const Vector3 direction(1, -2, 0);
    const Density density = 3 * direction * direction.transpose();
    const auto result = KalmanPredictor::predict(initial(), {0, Time{2'000'000'000}}, density);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const Covariance covariance = std::get<EstimatorState>(result).covariance();
    const Density shape = direction * direction.transpose();
    EXPECT_EQ((covariance.topLeftCorner<3, 3>()), (8 * shape).eval());
    EXPECT_EQ((covariance.topRightCorner<3, 3>()), (6 * shape).eval());
    EXPECT_EQ((covariance.bottomLeftCorner<3, 3>()), (6 * shape).eval());
    EXPECT_EQ((covariance.bottomRightCorner<3, 3>()), (6 * shape).eval());
}

TEST(KalmanPredictorTest, PreservesMixedScalesAndOneNanosecondAtEndOfRange) {
    StateVector vector = StateVector::Zero();
    vector(3) = 1e9;
    Covariance covariance = Covariance::Zero();
    covariance(0, 0) = 1e280;
    covariance(5, 5) = 1e-280;
    Density density = Density::Zero();
    density(0, 0) = 1e280;
    density(2, 2) = 1e-280;
    const auto start = initial(vector, covariance, {42, Time::max() - Time{1}});
    const auto result = KalmanPredictor::predict(start, {42, Time::max()}, density);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& estimate = std::get<EstimatorState>(result);
    EXPECT_DOUBLE_EQ(estimate.positionWorldM().x(), 1.0);
    EXPECT_DOUBLE_EQ(estimate.covariance()(0, 0), 1e280);
    EXPECT_DOUBLE_EQ(estimate.covariance()(3, 3), 1e271);
    EXPECT_DOUBLE_EQ(estimate.covariance()(5, 5), 1e-280 + 1e-289);
}

TEST(KalmanPredictorTest, AcceptsLargestElapsedDifference) {
    StateVector vector = StateVector::Zero();
    vector(4) = 1;
    const auto result = KalmanPredictor::predict(initial(vector), {0, Time::max()}, Density::Zero());
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& estimate = std::get<EstimatorState>(result);
    EXPECT_DOUBLE_EQ(estimate.positionWorldM().y(), 9223372036.854776);
    EXPECT_EQ(estimate.timestamp().elapsed, Time::max());
}

TEST(KalmanPredictorTest, RejectsInvalidEpochAndNonForwardTargets) {
    const auto start = initial(StateVector::Zero(), Covariance::Zero(), {2, Time{10}});
    const Density noise = Density::Zero();
    expectError(KalmanPredictor::predict(start, {2, Time{-1}}, noise),
                PredictionError::InvalidTargetTimestamp);
    expectError(KalmanPredictor::predict(start, {3, Time{11}}, noise),
                PredictionError::EpochMismatch);
    expectError(KalmanPredictor::predict(start, {2, Time{10}}, noise),
                PredictionError::NonForwardTime);
    expectError(KalmanPredictor::predict(start, {2, Time{9}}, noise),
                PredictionError::NonForwardTime);
}

TEST(KalmanPredictorTest, RejectsNonfiniteAsymmetricAndIndefiniteNoise) {
    Density noise = Density::Identity();
    const auto start = initial();
    const SimulationTimestamp target{0, Time{1}};
    noise(0, 1) = 0.25;
    expectError(KalmanPredictor::predict(start, target, noise), PredictionError::InvalidProcessNoise);
    noise(1, 0) = noise(0, 1) = 2;
    expectError(KalmanPredictor::predict(start, target, noise), PredictionError::InvalidProcessNoise);
    noise(1, 0) = noise(0, 1) = std::numeric_limits<double>::quiet_NaN();
    expectError(KalmanPredictor::predict(start, target, noise), PredictionError::InvalidProcessNoise);
    noise(1, 0) = noise(0, 1) = std::numeric_limits<double>::infinity();
    expectError(KalmanPredictor::predict(start, target, noise), PredictionError::InvalidProcessNoise);
}

TEST(KalmanPredictorTest, RejectsStateCovarianceAndProcessNoiseOverflow) {
    const double largest = std::numeric_limits<double>::max();
    const SimulationTimestamp target{0, Time{2'000'000'000}};
    StateVector vector = StateVector::Zero();
    vector(3) = largest;
    expectError(KalmanPredictor::predict(initial(vector), target, Density::Zero()),
                PredictionError::NumericalOverflow);
    Covariance covariance = Covariance::Zero();
    covariance(3, 3) = largest;
    expectError(KalmanPredictor::predict(initial(StateVector::Zero(), covariance), target,
                                        Density::Zero()), PredictionError::NumericalOverflow);
    expectError(KalmanPredictor::predict(initial(), target, largest * Density::Identity()),
                PredictionError::NumericalOverflow);
}

TEST(KalmanPredictorTest, RecoversRepresentableCancellationWithoutChangingOtherAxes) {
    const double largest = std::numeric_limits<double>::max();
    StateVector vector = StateVector::Zero();
    vector(0) = -largest;
    vector(3) = largest;
    Covariance covariance = Covariance::Zero();
    covariance(0, 0) = covariance(3, 3) = largest;
    covariance(0, 3) = covariance(3, 0) = -largest;
    covariance(1, 1) = std::numeric_limits<double>::denorm_min();
    const auto start = initial(vector, covariance);
    const auto result = KalmanPredictor::predict(start, {0, Time{2'000'000'000}}, Density::Zero());
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& estimate = std::get<EstimatorState>(result);
    EXPECT_EQ(estimate.positionWorldM().x(), largest);
    EXPECT_EQ(estimate.covariance()(0, 0), largest);
    EXPECT_EQ(estimate.covariance()(0, 3), largest);
    EXPECT_EQ(estimate.covariance()(1, 1), covariance(1, 1));
}

TEST(KalmanPredictorTest, PreservesRepresentableSubnormalCovariance) {
    const double s = std::numeric_limits<double>::denorm_min();
    Covariance covariance = Covariance::Zero();
    covariance(0, 0) = s;
    covariance(0, 3) = covariance(3, 0) = -2 * s;
    covariance(3, 3) = 16 * s;
    const auto input = EstimatorState::create(StateVector::Zero(), covariance, {});
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(input));
    const auto& start = std::get<EstimatorState>(input);
    Covariance expected = covariance;
    expected(0, 3) = expected(3, 0) = 2 * s;
    const SimulationTimestamp target{0, Time{250'000'000}};
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(
        EstimatorState::create(StateVector::Zero(), expected, target)));

    const auto result = KalmanPredictor::predict(start, target, Density::Zero());
    EXPECT_EQ(start.covariance(), covariance);
    EXPECT_EQ(start.vector(), StateVector::Zero());
    EXPECT_EQ(start.timestamp(), SimulationTimestamp{});
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& predicted = std::get<EstimatorState>(result);
    EXPECT_EQ(predicted.covariance(), expected);
    EXPECT_EQ(predicted.timestamp(), target);
}

TEST(KalmanPredictorTest, PreservesMixedScaleCancellationToExactZero) {
    const double s = std::numeric_limits<double>::denorm_min();
    Covariance covariance = 10 * Covariance::Identity();
    covariance(1, 0) = covariance(0, 1) = -3;
    covariance(1, 3) = covariance(3, 1) = 4;
    covariance(4, 3) = covariance(3, 4) = 2 * s;
    Density noise = 10 * Density::Identity();
    noise(1, 0) = noise(0, 1) = -8 * s;
    const auto input = EstimatorState::create(StateVector::Zero(), covariance, {});
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(input));
    const auto& start = std::get<EstimatorState>(input);
    const SimulationTimestamp target{0, Time{750'000'000}};

    // Success also verifies the process noise and full output covariance contracts.
    const auto result = KalmanPredictor::predict(start, target, noise);
    EXPECT_EQ(start.covariance(), covariance);
    EXPECT_EQ(start.vector(), StateVector::Zero());
    EXPECT_EQ(start.timestamp(), SimulationTimestamp{});
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& predicted = std::get<EstimatorState>(result);
    EXPECT_EQ(predicted.covariance()(1, 0), 0.0);
    EXPECT_EQ(predicted.covariance()(0, 1), 0.0);
    EXPECT_EQ(predicted.timestamp(), target);
}

TEST(KalmanPredictorTest, RejectsInvalidPropagationOfToleranceValidInitialCovariance) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 3) = covariance(3, 0) = -(1 + 2e-13);
    const auto start = initial(StateVector::Zero(), covariance);
    expectError(KalmanPredictor::predict(start, {0, Time{1'000'000'000}}, Density::Zero()),
                PredictionError::InvalidPropagatedCovariance);
    EXPECT_EQ(start.covariance(), covariance);
    EXPECT_EQ(start.vector(), StateVector::Zero());
    EXPECT_EQ(start.timestamp(), SimulationTimestamp{});
}

TEST(KalmanPredictorTest, ReplaysExactlyAndComposesWithinRoundingTolerance) {
    const auto start = initial(StateVector::LinSpaced(1.0, 6.0), Covariance::Identity());
    Density noise;
    noise << 2, 0.5, -0.25, 0.5, 1, 0, -0.25, 0, 3;
    const SimulationTimestamp end{0, Time{1'900'000'000}};
    const auto once = KalmanPredictor::predict(start, end, noise);
    const auto replay = KalmanPredictor::predict(start, end, noise);
    const auto first = KalmanPredictor::predict(start, {0, Time{700'000'000}}, noise);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(once));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(replay));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(first));
    const auto twice = KalmanPredictor::predict(std::get<EstimatorState>(first), end, noise);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(twice));
    const auto& expected = std::get<EstimatorState>(once);
    EXPECT_EQ(expected.vector(), std::get<EstimatorState>(replay).vector());
    EXPECT_EQ(expected.covariance(), std::get<EstimatorState>(replay).covariance());
    // A few dozen rounding steps on order-one inputs across both paths.
    EXPECT_TRUE(expected.vector().isApprox(std::get<EstimatorState>(twice).vector(), 1e-14));
    EXPECT_TRUE(expected.covariance().isApprox(std::get<EstimatorState>(twice).covariance(), 1e-14));
    EXPECT_EQ(expected.timestamp(), std::get<EstimatorState>(twice).timestamp());
}

} // namespace
} // namespace aerotwin
