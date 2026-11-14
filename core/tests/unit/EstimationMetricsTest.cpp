#include <aerotwin/validation/EstimationMetrics.hpp>

#include <aerotwin/estimation/GpsKalmanUpdater.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

namespace aerotwin {
namespace {

EstimateTruthPair sample(const Vector3& position = Vector3::Zero(),
                         const Vector3& velocity = Vector3::Zero(),
                         SimulationTimestamp estimate_time = {},
                         SimulationTimestamp truth_time = {},
                         const Vector3& truth_position = Vector3::Zero()) {
    EstimatorVector6 state;
    state << position, velocity;
    return {
        std::get<EstimatorState>(EstimatorState::create(
            state, EstimatorCovariance6::Zero(), estimate_time)),
        VehicleState::create(truth_position, Quaternion::Identity(), Vector3::Zero(),
                             Vector3::Zero(), truth_time).value(),
    };
}

void expectFailure(std::span<const EstimateTruthPair> samples, EvaluationError error,
                   std::optional<std::size_t> index) {
    const auto result = computeRmse(samples);
    ASSERT_TRUE(std::holds_alternative<RmseFailure>(result));
    const auto& failure = std::get<RmseFailure>(result);
    EXPECT_EQ(failure.error, error);
    EXPECT_EQ(failure.sample_index, index);
}

TEST(EstimationMetricsTest, IdenticalSnapshotsProduceZeroError) {
    const std::array samples{sample(Vector3::Ones(), Vector3::Zero(), {}, {}, Vector3::Ones())};
    const auto result = computeRmse(samples);
    ASSERT_TRUE(std::holds_alternative<RmseResult>(result));
    const auto& rmse = std::get<RmseResult>(result);
    EXPECT_EQ(rmse.sample_count, 1u);
    EXPECT_EQ(rmse.position_axis_m, Vector3::Zero());
    EXPECT_EQ(rmse.velocity_axis_mps, Vector3::Zero());
    EXPECT_EQ(rmse.position_3d_m, 0.0);
    EXPECT_EQ(rmse.velocity_3d_mps, 0.0);
}

TEST(EstimationMetricsTest, DistinguishesPerAxisAndSpatialRmse) {
    const SimulationTimestamp later{0, SimulationNanoseconds{900}};
    const std::array samples{
        sample(Vector3(3, 4, 0), Vector3(0, 0, 2), later, later),
        sample(Vector3::Zero(), Vector3(0, 0, 4)),
    };
    const auto result = computeRmse(samples);
    ASSERT_TRUE(std::holds_alternative<RmseResult>(result));
    const auto& rmse = std::get<RmseResult>(result);
    EXPECT_EQ(rmse.sample_count, 2u);
    EXPECT_DOUBLE_EQ(rmse.position_axis_m.x(), 3 / std::sqrt(2.0));
    EXPECT_DOUBLE_EQ(rmse.position_axis_m.y(), 4 / std::sqrt(2.0));
    EXPECT_EQ(rmse.position_axis_m.z(), 0.0);
    EXPECT_DOUBLE_EQ(rmse.position_3d_m, 5 / std::sqrt(2.0));
    EXPECT_EQ(rmse.velocity_axis_mps.x(), 0.0);
    EXPECT_EQ(rmse.velocity_axis_mps.y(), 0.0);
    EXPECT_DOUBLE_EQ(rmse.velocity_axis_mps.z(), std::sqrt(10.0));
    EXPECT_DOUBLE_EQ(rmse.velocity_3d_mps, std::sqrt(10.0));
}

TEST(EstimationMetricsTest, RejectsEmptyBatch) {
    expectFailure({}, EvaluationError::EmptySamples, std::nullopt);
}

TEST(EstimationMetricsTest, ReportsPairAndBatchEpochMismatches) {
    const SimulationTimestamp other_epoch{1, {}};
    const std::array pair_mismatch{sample(Vector3::Zero(), Vector3::Zero(), {}, other_epoch)};
    expectFailure(pair_mismatch, EvaluationError::EpochMismatch, 0);
    const std::array batch_mismatch{
        sample(), sample(Vector3::Zero(), Vector3::Zero(), other_epoch, other_epoch)};
    expectFailure(batch_mismatch, EvaluationError::EpochMismatch, 1);
}

TEST(EstimationMetricsTest, ReportsFirstInvalidSampleInInputOrder) {
    const SimulationTimestamp last{0, SimulationNanoseconds::max()};
    const SimulationTimestamp earlier{0, last.elapsed - SimulationNanoseconds{1}};
    const std::array samples{
        sample(Vector3::Ones()), sample(Vector3::Zero(), Vector3::Zero(), last, earlier),
        sample(Vector3::Zero(), Vector3::Zero(), {}, {1, {}}),
    };
    expectFailure(samples, EvaluationError::TimestampMismatch, 1);
}

TEST(EstimationMetricsTest, PreservesLargeSmallAndSubnormalErrors) {
    const double smallest = std::numeric_limits<double>::denorm_min();
    const std::array samples{sample(Vector3(1e200, -1e-200, smallest),
                                    Vector3(1e-200, -1e200, -smallest))};
    const auto result = computeRmse(samples);
    ASSERT_TRUE(std::holds_alternative<RmseResult>(result));
    const auto& rmse = std::get<RmseResult>(result);
    EXPECT_EQ(rmse.position_axis_m, Vector3(1e200, 1e-200, smallest));
    EXPECT_EQ(rmse.velocity_axis_mps, Vector3(1e-200, 1e200, smallest));
    EXPECT_EQ(rmse.position_3d_m, 1e200);
    EXPECT_EQ(rmse.velocity_3d_mps, 1e200);
}

TEST(EstimationMetricsTest, ComputesRepresentableMeanFromOversizedDifference) {
    const double largest = std::numeric_limits<double>::max();
    const auto wide = sample(Vector3(largest, 0, 0), Vector3::Zero(), {}, {},
                             Vector3(-largest, 0, 0));
    const std::array samples{wide, sample(), sample(), sample()};
    const auto result = computeRmse(samples);
    ASSERT_TRUE(std::holds_alternative<RmseResult>(result));
    const auto& rmse = std::get<RmseResult>(result);
    EXPECT_EQ(rmse.position_axis_m.x(), largest);
    EXPECT_EQ(rmse.position_3d_m, largest);
    expectFailure(std::span(samples).first(1), EvaluationError::NumericalFailure, std::nullopt);
}

TEST(EstimationMetricsTest, RejectsUnrepresentableSpatialAndPositiveUnderflowResults) {
    const double largest = std::numeric_limits<double>::max();
    const std::array spatial{sample(Vector3(largest, largest, 0))};
    expectFailure(spatial, EvaluationError::NumericalFailure, std::nullopt);

    const double smallest = std::numeric_limits<double>::denorm_min();
    const std::array tiny{sample(Vector3(smallest, 0, 0)), sample(), sample(), sample(), sample()};
    expectFailure(tiny, EvaluationError::NumericalFailure, std::nullopt);
}

TEST(EstimationMetricsTest, CountsRepeatedPairsAndReplaysWithoutMutation) {
    const auto nonzero = sample(Vector3(3, 0, 0), Vector3(0, 0, 4), {}, {}, Vector3::Ones());
    const std::array samples{nonzero, sample(), nonzero};
    const auto before = samples;
    const auto first = computeRmse(samples);
    const auto second = computeRmse(samples);
    ASSERT_TRUE(std::holds_alternative<RmseResult>(first));
    ASSERT_TRUE(std::holds_alternative<RmseResult>(second));
    const auto& a = std::get<RmseResult>(first);
    const auto& b = std::get<RmseResult>(second);
    EXPECT_EQ(a.sample_count, 3u);
    EXPECT_DOUBLE_EQ(a.position_3d_m, 2.0);
    EXPECT_EQ(a.sample_count, b.sample_count);
    EXPECT_EQ(a.position_axis_m, b.position_axis_m);
    EXPECT_EQ(a.velocity_axis_mps, b.velocity_axis_mps);
    EXPECT_EQ(a.position_3d_m, b.position_3d_m);
    EXPECT_EQ(a.velocity_3d_mps, b.velocity_3d_mps);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        EXPECT_EQ(samples[i].estimate.vector(), before[i].estimate.vector());
        EXPECT_EQ(samples[i].estimate.covariance(), before[i].estimate.covariance());
        EXPECT_EQ(samples[i].estimate.timestamp(), before[i].estimate.timestamp());
        EXPECT_EQ(samples[i].truth.positionWorldM(), before[i].truth.positionWorldM());
        EXPECT_EQ(samples[i].truth.linearVelocityWorldMps(), before[i].truth.linearVelocityWorldMps());
        EXPECT_EQ(samples[i].truth.timestamp(), before[i].truth.timestamp());
    }
}

using Components = NeesComponents;
using Covariance = EstimatorCovariance6;

EstimateTruthPair neesSample(const Covariance& covariance,
                             const Vector3& position = Vector3::Zero(),
                             const Vector3& velocity = Vector3::Zero()) {
    auto pair = sample(position, velocity);
    pair.estimate = std::get<EstimatorState>(
        EstimatorState::create(pair.estimate.vector(), covariance, pair.estimate.timestamp()));
    return pair;
}

void expectNees(const EstimateTruthPair& pair, Components components, double value, unsigned dof) {
    const auto result = computeNees(pair.estimate, pair.truth, components);
    ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(result));
    const auto score = std::get<ConsistencyScore>(result);
    EXPECT_DOUBLE_EQ(score.value, value);
    EXPECT_EQ(score.degrees_of_freedom, dof);
}

void expectNeesError(const EstimateTruthPair& pair, Components components, EvaluationError error) {
    const auto result = computeNees(pair.estimate, pair.truth, components);
    ASSERT_TRUE(std::holds_alternative<EvaluationError>(result));
    EXPECT_EQ(std::get<EvaluationError>(result), error);
}

TEST(NeesTest, ZeroResidualWithPositiveDefiniteCovariance) {
    const auto pair = neesSample(Covariance::Identity());
    expectNees(pair, Components::FullState, 0, 6);
    expectNees(pair, Components::Position, 0, 3);
    expectNees(pair, Components::Velocity, 0, 3);
}

TEST(NeesTest, AnalyticalDiagonalScoreReplaysWithoutMutation) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = 4;
    covariance(3, 3) = 9;
    const auto pair = neesSample(covariance, Vector3(2, 0, 0), Vector3(3, 0, 0));
    expectNees(pair, Components::FullState, 2, 6);
    expectNees(pair, Components::Position, 1, 3);
    expectNees(pair, Components::Velocity, 1, 3);
    const auto first = std::get<ConsistencyScore>(
        computeNees(pair.estimate, pair.truth, Components::FullState));
    const auto second = std::get<ConsistencyScore>(
        computeNees(pair.estimate, pair.truth, Components::FullState));
    EXPECT_EQ(first.value, second.value);
    EXPECT_EQ(first.degrees_of_freedom, second.degrees_of_freedom);
    EXPECT_EQ(pair.estimate.covariance(), covariance);
    EXPECT_EQ(pair.estimate.positionWorldM(), Vector3(2, 0, 0));
    EXPECT_EQ(pair.estimate.velocityWorldMps(), Vector3(3, 0, 0));
    EXPECT_EQ(pair.truth.positionWorldM(), Vector3::Zero());
    EXPECT_EQ(pair.truth.linearVelocityWorldMps(), Vector3::Zero());
    EXPECT_EQ(pair.estimate.timestamp(), SimulationTimestamp{});
    EXPECT_EQ(pair.truth.timestamp(), SimulationTimestamp{});
}

TEST(NeesTest, FullScoreUsesPositionVelocityCorrelation) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 3) = covariance(3, 0) = 0.5;
    const auto pair = neesSample(covariance, Vector3::UnitX(), Vector3::UnitX());
    expectNees(pair, Components::FullState, 4.0 / 3.0, 6);
    expectNees(pair, Components::Position, 1, 3);
    expectNees(pair, Components::Velocity, 1, 3);
}

TEST(NeesTest, PositionScoreUsesCorrelatedMarginal) {
    Covariance covariance = Covariance::Identity();
    covariance.topLeftCorner<3, 3>() << 4, 2, 0, 2, 4, 0, 0, 0, 1;
    // (2, 2, 0) lies along a mode with variance 6.
    expectNees(neesSample(covariance, Vector3(2, 2, 0)), Components::Position, 4.0 / 3.0, 3);
}

TEST(NeesTest, RejectsSingularSelectionEvenWithoutError) {
    Covariance covariance = Covariance::Identity();
    covariance(5, 5) = 0;
    const auto pair = neesSample(covariance, Vector3::UnitX());
    expectNeesError(pair, Components::FullState, EvaluationError::UnsolvableCovariance);
    expectNees(pair, Components::Position, 1, 3);
    expectNeesError(pair, Components::Velocity, EvaluationError::UnsolvableCovariance);
    expectNeesError(neesSample(covariance), Components::FullState,
                    EvaluationError::UnsolvableCovariance);
}

TEST(NeesTest, RejectsNumericallyUnsolvableCorrelation) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 1) = covariance(1, 0) = std::nextafter(1.0, 0.0);
    expectNeesError(neesSample(covariance), Components::Position,
                    EvaluationError::UnsolvableCovariance);
    covariance(0, 1) = covariance(1, 0) = 1.0 + 2e-13;
    expectNeesError(neesSample(covariance), Components::Position,
                    EvaluationError::UnsolvableCovariance);
    covariance(0, 1) = covariance(1, 0) = 1.0 - 1e-8;
    expectNees(neesSample(covariance), Components::Position, 0, 3);
}

TEST(NeesTest, RejectsEpochAndSingleNanosecondMismatches) {
    const auto last = SimulationNanoseconds::max();
    expectNeesError(sample(Vector3::Zero(), Vector3::Zero(), {1, last}, {2, last}),
                    Components::FullState, EvaluationError::EpochMismatch);
    expectNeesError(sample(Vector3::Zero(), Vector3::Zero(), {1, last}, {1, last - SimulationNanoseconds{1}}),
                    Components::FullState, EvaluationError::TimestampMismatch);
}

TEST(NeesTest, ScoresMixedScaleAndSubnormalCovariances) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = 0x1p-1000;
    covariance(2, 2) = 0x1p1000;
    expectNees(neesSample(covariance, Vector3(0x1p-500, 1, 0x1p500)), Components::Position, 3, 3);

    covariance(0, 0) = std::numeric_limits<double>::denorm_min();
    expectNees(neesSample(covariance, Vector3(0x1p-537, 0, 0)), Components::Position, 1, 3);
    expectNees(neesSample(Covariance::Identity(), Vector3(0x1p-537, 0, 0)),
               Components::Position, std::numeric_limits<double>::denorm_min(), 3);
}

TEST(NeesTest, AppliesPivotPermutationForCorrelatedCovariance) {
    Covariance covariance = Covariance::Identity();
    covariance.topLeftCorner<3, 3>() << 0.5, 0.5, 0, 0.5, 3, 0, 0, 0, 1;
    // The residual equals the first column of the selected covariance.
    expectNees(neesSample(covariance, Vector3(0.5, 0.5, 0)), Components::Position, 0.5, 3);
}

TEST(NeesTest, PreservesFiniteScoreAtUpperBoundary) {
    const SimulationTimestamp time{7, SimulationNanoseconds{123}};
    auto pair = sample(Vector3(0x1p512, 0, 0), Vector3::Zero(), time, time);
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = std::nextafter(1.0, 2.0);
    // Also exercise a correlated y/z block independent of the extreme x axis.
    for (double correlation : {0.0, 0.5}) {
        covariance(1, 2) = covariance(2, 1) = correlation;
        const auto state = pair.estimate.vector();
        pair.estimate = std::get<EstimatorState>(EstimatorState::create(state, covariance, time));
        const auto result = computeNees(pair.estimate, pair.truth, Components::Position);
        EXPECT_TRUE(std::holds_alternative<ConsistencyScore>(result));
        if (const auto* score = std::get_if<ConsistencyScore>(&result)) {
            EXPECT_EQ(score->value, 0x1.ffffffffffffep+1023);
            EXPECT_EQ(score->degrees_of_freedom, 3u);
        }
        EXPECT_EQ(pair.estimate.vector(), state);
        EXPECT_EQ(pair.estimate.covariance(), covariance);
        EXPECT_EQ(pair.estimate.timestamp(), time);
        EXPECT_EQ(pair.truth.timestamp(), time);
        EXPECT_EQ(pair.truth.positionWorldM(), Vector3::Zero());
        EXPECT_EQ(pair.truth.linearVelocityWorldMps(), Vector3::Zero());
    }
}

TEST(NeesTest, PreservesFiniteScoreWhenMultipleTermsApproachOverflow) {
    const SimulationTimestamp time{7, SimulationNanoseconds{123}};
    const Vector3 position(0x1p512, 0x1.2p485, 0x1.2p485);
    auto pair = sample(position, Vector3::Zero(), time, time);
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = std::nextafter(1.0, 2.0);
    const auto state = pair.estimate.vector();
    pair.estimate = std::get<EstimatorState>(EstimatorState::create(state, covariance, time));

    const auto result = computeNees(pair.estimate, pair.truth, Components::Position);
    ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(result));
    const auto score = std::get<ConsistencyScore>(result);
    EXPECT_EQ(score.value, std::numeric_limits<double>::max());
    EXPECT_EQ(score.degrees_of_freedom, 3u);
    EXPECT_EQ(pair.estimate.vector(), state);
    EXPECT_EQ(pair.estimate.covariance(), covariance);
    EXPECT_EQ(pair.estimate.timestamp(), time);
    EXPECT_EQ(pair.truth.positionWorldM(), Vector3::Zero());
    EXPECT_EQ(pair.truth.linearVelocityWorldMps(), Vector3::Zero());
    EXPECT_EQ(pair.truth.timestamp(), time);
}

TEST(NeesTest, RejectsMultiTermScoreAboveOverflowRoundingThreshold) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = std::nextafter(1.0, 2.0);
    expectNeesError(neesSample(covariance, Vector3(0x1p512, 0x1.4p485, 0x1.4p485)),
                    Components::Position, EvaluationError::NumericalFailure);
}

TEST(NeesTest, PreservesSquaredContributionAboveSubnormalMidpoint) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = 0x1.0000000000002p+1;
    // The positive square remainder puts the score above half a subnormal unit.
    const auto pair = neesSample(covariance, Vector3(0x1.0000000000001p-537, 0, 0));
    const auto result = computeNees(pair.estimate, pair.truth, Components::Position);
    ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(result));
    const auto score = std::get<ConsistencyScore>(result);
    EXPECT_EQ(score.value, std::numeric_limits<double>::denorm_min());
    EXPECT_EQ(score.degrees_of_freedom, 3u);
}

TEST(NeesTest, RoundsSubnormalScoreUsingCompensatedRemainder) {
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = 0.5;
    constexpr auto mode = Components::Position;
    const double unit = std::numeric_limits<double>::denorm_min();
    const auto above = neesSample(covariance, Vector3(0x1p-538, 0x1p-565, 0));
    const auto result = computeNees(above.estimate, above.truth, mode);
    ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(result));
    EXPECT_EQ(std::get<ConsistencyScore>(result).value, unit);
    EXPECT_EQ(std::get<ConsistencyScore>(result).degrees_of_freedom, 3u);
    expectNeesError(neesSample(covariance, Vector3(0x1p-538, 0, 0)), mode,
                    EvaluationError::NumericalFailure);
    expectNeesError(neesSample(covariance, Vector3(0x1p-539, 0, 0)), mode,
                    EvaluationError::NumericalFailure);
    expectNees(neesSample(Covariance::Identity(), Vector3(0x1p-511, 0, 0)),
               mode, std::numeric_limits<double>::min(), 3);
}

TEST(NeesTest, RejectsUnrepresentableScoreFromOversizedDifference) {
    const double position = 0x1p1023;
    auto pair = sample(Vector3(position, 0, 0), Vector3::Zero(), {}, {}, Vector3(-position, 0, 0));
    Covariance covariance = Covariance::Identity();
    covariance(0, 0) = 0x1p1020;
    pair.estimate = std::get<EstimatorState>(
        EstimatorState::create(pair.estimate.vector(), covariance, {}));
    // This difference is 2^1024 and its score is also outside the range of double.
    expectNeesError(pair, Components::Position, EvaluationError::NumericalFailure);
}

TEST(NeesTest, RejectsOverflowUnderflowAndUnsupportedRhsRange) {
    expectNeesError(neesSample(Covariance::Identity(), Vector3(1e200, 0, 0)),
                    Components::Position, EvaluationError::NumericalFailure);
    expectNeesError(neesSample(Covariance::Identity(), Vector3(0x1p-538, 0, 0)),
                    Components::Position, EvaluationError::NumericalFailure);
    expectNeesError(neesSample(Covariance::Identity(),
                               Vector3(1, std::numeric_limits<double>::denorm_min(), 0)),
                    Components::Position, EvaluationError::NumericalFailure);
}

using Matrix3 = Eigen::Matrix3d;

GpsMeasurement observation(const Vector3& position, const Matrix3& covariance,
                           SimulationTimestamp time = {}) {
    return GpsMeasurement::create(MeasurementAcquisition::create("gps", time).value(),
                                  position, covariance).value();
}

void expectNis(const EstimatorState& prior, const GpsMeasurement& measurement, double expected) {
    const auto result = computeGpsNis(prior, measurement);
    ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(result));
    EXPECT_DOUBLE_EQ(std::get<ConsistencyScore>(result).value, expected);
    EXPECT_EQ(std::get<ConsistencyScore>(result).degrees_of_freedom, 3u);
}

void expectNisError(const EstimatorState& prior, const GpsMeasurement& measurement,
                    EvaluationError expected) {
    const auto result = computeGpsNis(prior, measurement);
    ASSERT_TRUE(std::holds_alternative<EvaluationError>(result));
    EXPECT_EQ(std::get<EvaluationError>(result), expected);
}

TEST(GpsNisTest, ScoresZeroInnovationWithPositiveDefiniteCovariance) {
    const auto pair = neesSample(Covariance::Identity(), Vector3(1, -2, 3));
    expectNis(pair.estimate, observation(Vector3(1, -2, 3), Matrix3::Identity()), 0.0);
}

TEST(GpsNisTest, ScoresPriorBeforeUpdateAndReplaysWithoutMutation) {
    const auto time = SimulationTimestamp{3, SimulationNanoseconds{50}};
    EstimatorVector6 state;
    state << 4, -3, 2, 9, 8, 7;
    const auto prior = std::get<EstimatorState>(
        EstimatorState::create(state, Covariance::Identity(), time));
    const Vector3 position(6, -2, 2);
    const auto measurement = observation(position, Matrix3::Identity(), time);
    expectNis(prior, measurement, 2.5);
    const auto update = GpsKalmanUpdater::update(prior, measurement);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(update));
    const auto& posterior = std::get<EstimatorState>(update);
    EXPECT_TRUE(posterior.positionWorldM().isApprox(Vector3(5, -2.5, 2), 1e-14));
    // Supplying the posterior instead changes both the residual and its scale.
    const auto posterior_score = computeGpsNis(posterior, measurement);
    ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(posterior_score));
    EXPECT_NEAR(std::get<ConsistencyScore>(posterior_score).value, 5.0 / 6.0, 1e-14);
    const auto first = std::get<ConsistencyScore>(computeGpsNis(prior, measurement));
    const auto second = std::get<ConsistencyScore>(computeGpsNis(prior, measurement));
    EXPECT_EQ(first.value, second.value);
    EXPECT_EQ(first.degrees_of_freedom, second.degrees_of_freedom);
    EXPECT_EQ(prior.vector(), state);
    EXPECT_EQ(prior.covariance(), Covariance::Identity());
    EXPECT_EQ(prior.timestamp(), time);
    EXPECT_EQ(measurement.positionWorldM(), position);
    EXPECT_EQ(measurement.positionCovarianceWorldM2(), Matrix3::Identity());
    EXPECT_EQ(measurement.acquisition().timestamp(), time);
    EXPECT_EQ(measurement.acquisition().sensorId(), "gps");
}

TEST(GpsNisTest, ScoresCorrelatedInnovationCovariance) {
    Matrix3 noise;
    noise << 3, 2, 0, 2, 3, 0, 0, 0, 0;
    expectNis(neesSample(Covariance::Identity()).estimate,
              observation(Vector3(2, 2, 0), noise), 4.0 / 3.0);
}

TEST(GpsNisTest, ScoresCorrelatedCovarianceWithPivotPermutation) {
    Matrix3 noise;
    noise << 0.5, 0.5, 0, 0.5, 3, 0, 0, 0, 1;
    // The innovation equals the first column of S, so the score is S(0,0).
    expectNis(neesSample(Covariance::Zero()).estimate,
              observation(Vector3(0.5, 0.5, 0), noise), 0.5);
}

TEST(GpsNisTest, RejectsSingularAndNumericallyRankDeficientCovariance) {
    const auto prior = neesSample(Covariance::Zero()).estimate;
    Matrix3 noise = Matrix3::Identity();
    noise(0, 1) = noise(1, 0) = 1.0;
    for (const Vector3 position : {Vector3(1, 0, 0), Vector3(0, 0, 0)}) {
        expectNisError(prior, observation(position, noise), EvaluationError::UnsolvableCovariance);
    }
    noise(0, 1) = noise(1, 0) = std::nextafter(1.0, 0.0);
    expectNisError(prior, observation(Vector3::Zero(), noise), EvaluationError::UnsolvableCovariance);
}

TEST(GpsNisTest, RequiresExactEpochAndNanoseconds) {
    const auto time = SimulationTimestamp{2, SimulationNanoseconds::max()};
    const auto prior = sample(Vector3::Zero(), Vector3::Zero(), time, time).estimate;
    expectNisError(prior, observation(Vector3::Zero(), Matrix3::Identity(), {3, time.elapsed}),
                   EvaluationError::EpochMismatch);
    expectNisError(prior, observation(Vector3::Zero(), Matrix3::Identity(),
                                     {2, time.elapsed - SimulationNanoseconds{1}}),
                   EvaluationError::TimestampMismatch);
}

TEST(GpsNisTest, ScoresMixedScaleCovariance) {
    Matrix3 noise = Matrix3::Identity();
    noise(0, 0) = 0x1p-1000;
    noise(2, 2) = 0x1p1000;
    expectNis(neesSample(Covariance::Zero()).estimate,
              observation(Vector3(0x1p-500, 1, 0x1p500), noise), 3.0);
}

TEST(GpsNisTest, PreservesLargeFiniteScoreAndRejectsGenuineOverflow) {
    const auto prior = neesSample(Covariance::Zero()).estimate;
    Matrix3 noise = Matrix3::Identity();
    noise(0, 0) = std::nextafter(1.0, 2.0);
    const auto finite = computeGpsNis(prior, observation(Vector3(0x1p512, 0, 0), noise));
    ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(finite));
    EXPECT_EQ(std::get<ConsistencyScore>(finite).value, 0x1.ffffffffffffep+1023);
    expectNisError(prior, observation(Vector3(0x1p512, 0, 0), Matrix3::Identity()),
                   EvaluationError::NumericalFailure);
}

TEST(GpsNisTest, PreservesSubnormalScoresAndRejectsUnderflowToZero) {
    const auto prior = neesSample(Covariance::Zero()).estimate;
    const double unit = std::numeric_limits<double>::denorm_min();
    auto expectUnit = [&](const Vector3& position, const Matrix3& noise) {
        const auto result = computeGpsNis(prior, observation(position, noise));
        ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(result));
        EXPECT_EQ(std::get<ConsistencyScore>(result).value, unit);
    };
    expectUnit(Vector3(0x1p-537, 0, 0), Matrix3::Identity());
    Matrix3 noise = Matrix3::Identity();
    noise(0, 0) = 0.5;
    expectUnit(Vector3(0x1p-538, 0x1p-565, 0), noise);
    expectNisError(prior, observation(Vector3(0x1p-538, 0, 0), noise),
                   EvaluationError::NumericalFailure);
    expectNisError(prior, observation(Vector3(0x1p-539, 0, 0), noise),
                   EvaluationError::NumericalFailure);
}

TEST(GpsNisTest, RejectsOverflowInRawInnovationAndCovarianceAddition) {
    const double largest = std::numeric_limits<double>::max();
    expectNisError(neesSample(Covariance::Identity(), Vector3(-largest, 0, 0)).estimate,
                   observation(Vector3(largest, 0, 0), Matrix3::Identity()),
                   EvaluationError::NumericalFailure);
    expectNisError(neesSample(largest * Covariance::Identity()).estimate,
                   observation(Vector3::Zero(), largest * Matrix3::Identity()),
                   EvaluationError::NumericalFailure);
}

} // namespace
} // namespace aerotwin
