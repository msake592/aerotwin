#include <aerotwin/estimation/GpsKalmanUpdater.hpp>
#include <aerotwin/estimation/KalmanPredictor.hpp>
#include <aerotwin/validation/EstimationMetrics.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

namespace aerotwin {
namespace {

using Time = SimulationNanoseconds;
using Matrix3 = Eigen::Matrix3d;
using Score = std::variant<ConsistencyScore, EvaluationError>;

EstimatorState initialEstimate(const EstimatorCovariance6& covariance) {
    return std::get<EstimatorState>(EstimatorState::create(
        EstimatorVector6::Zero(), covariance, {7, Time{0}}));
}

VehicleState truthAt(SimulationTimestamp time, const Vector3& position) {
    return VehicleState::create(position, Quaternion::Identity(), Vector3(1, 0.5, 0),
                                Vector3::Zero(), time).value();
}

GpsMeasurement gpsAt(SimulationTimestamp time, const Vector3& position, double variance) {
    return GpsMeasurement::create(MeasurementAcquisition::create("gps", time).value(),
                                  position, variance * Matrix3::Identity()).value();
}

void expectScore(const Score& result, double expected, unsigned dof) {
    ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(result));
    const auto score = std::get<ConsistencyScore>(result);
    EXPECT_NEAR(score.value, expected, 1e-13);
    EXPECT_EQ(score.degrees_of_freedom, dof);
}

struct EvaluationRun {
    std::vector<EstimateTruthPair> pairs;
    std::vector<ConsistencyScore> scores;
    std::optional<RmseResult> rmse;
};

void runEvaluation(EvaluationRun& run) {
    using Components = NeesComponents;
    const auto initial = initialEstimate(EstimatorCovariance6::Identity());
    auto current = initial;
    const std::array times{SimulationTimestamp{7, Time{1'000'000'000}},
                           SimulationTimestamp{7, Time{2'000'000'000}}};
    const std::array measurements{gpsAt(times[0], Vector3(2, 0, 0), 2),
                                  gpsAt(times[1], Vector3(3.5, 0, 0), 11.0 / 4)};
    const std::array truths{truthAt(times[0], Vector3(1, 1, 0)),
                            truthAt(times[1], Vector3(2, 1.5, 0))};
    const auto original_measurements = measurements;
    const auto original_truths = truths;
    const std::array expected_positions{Vector3(1, 0, 0), Vector3(2.5, 0, 0)};
    const std::array expected_velocities{Vector3(0.5, 0, 0), Vector3(21.0 / 22, 0, 0)};
    const std::array expected_nis{1.0, 8.0 / 11};
    const std::array expected_nees{std::array{1.5, 1.0, 2.0 / 3},
                                  std::array{53.0 / 22, 20.0 / 11, 244.0 / 451}};
    for (std::size_t i = 0; i < times.size(); ++i) {
        SCOPED_TRACE(i);
        const auto prediction = KalmanPredictor::predict(current, times[i], Matrix3::Zero());
        ASSERT_TRUE(std::holds_alternative<EstimatorState>(prediction));
        const auto& prior = std::get<EstimatorState>(prediction);
        const auto saved_prior = prior;
        EXPECT_EQ(prior.timestamp(), measurements[i].acquisition().timestamp());
        const auto nis = computeGpsNis(prior, measurements[i]);
        ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(nis));
        expectScore(nis, expected_nis[i], 3);
        run.scores.push_back(std::get<ConsistencyScore>(nis));

        const auto update = GpsKalmanUpdater::update(prior, measurements[i]);
        ASSERT_TRUE(std::holds_alternative<EstimatorState>(update));
        const auto& posterior = std::get<EstimatorState>(update);
        EXPECT_EQ(posterior.timestamp(), times[i]);
        EXPECT_EQ(posterior.timestamp(), truths[i].timestamp());
        EXPECT_TRUE(posterior.positionWorldM().isApprox(expected_positions[i], 1e-13));
        EXPECT_TRUE(posterior.velocityWorldMps().isApprox(expected_velocities[i], 1e-13));
        EXPECT_NEAR(posterior.covariance()(0, 3), i == 0 ? 0.5 : 5.0 / 8, 1e-13);
        if (i == 0) {
            // Diagnostic contrast only. The collected NIS series uses the prior.
            expectScore(computeGpsNis(posterior, measurements[i]), 1.0 / 3, 3);
        }

        const std::array modes{Components::FullState, Components::Position, Components::Velocity};
        for (std::size_t mode = 0; mode < modes.size(); ++mode) {
            const auto nees = computeNees(posterior, truths[i], modes[mode]);
            ASSERT_TRUE(std::holds_alternative<ConsistencyScore>(nees));
            expectScore(nees, expected_nees[i][mode], mode == 0 ? 6 : 3);
            run.scores.push_back(std::get<ConsistencyScore>(nees));
        }
        run.pairs.push_back({posterior, truths[i]});
        EXPECT_EQ(prior.vector(), saved_prior.vector());
        EXPECT_EQ(prior.covariance(), saved_prior.covariance());
        EXPECT_EQ(prior.timestamp(), saved_prior.timestamp());
        EXPECT_EQ(measurements[i].positionWorldM(), original_measurements[i].positionWorldM());
        EXPECT_EQ(measurements[i].positionCovarianceWorldM2(),
                  original_measurements[i].positionCovarianceWorldM2());
        EXPECT_EQ(measurements[i].acquisition().timestamp(),
                  original_measurements[i].acquisition().timestamp());
        EXPECT_EQ(measurements[i].acquisition().sensorId(),
                  original_measurements[i].acquisition().sensorId());
        current = posterior;
    }

    const auto result = computeRmse(run.pairs);
    ASSERT_TRUE(std::holds_alternative<RmseResult>(result));
    run.rmse = std::get<RmseResult>(result);
    EXPECT_EQ(run.rmse->sample_count, 2u);
    EXPECT_TRUE(run.rmse->position_axis_m.isApprox(Vector3(std::sqrt(1.0 / 8),
                                                           std::sqrt(13.0 / 8), 0), 1e-13));
    EXPECT_TRUE(run.rmse->velocity_axis_mps.isApprox(Vector3(std::sqrt(61.0 / 484), 0.5, 0),
                                                            1e-13));
    EXPECT_NEAR(run.rmse->position_3d_m, std::sqrt(7.0 / 4), 1e-13);
    EXPECT_NEAR(run.rmse->velocity_3d_mps, std::sqrt(91.0 / 242), 1e-13);
    for (std::size_t i = 0; i < times.size(); ++i) {
        EXPECT_EQ(run.pairs[i].truth.positionWorldM(), original_truths[i].positionWorldM());
        EXPECT_EQ(run.pairs[i].truth.linearVelocityWorldMps(),
                  original_truths[i].linearVelocityWorldMps());
        EXPECT_EQ(run.pairs[i].truth.timestamp(), times[i]);
        EXPECT_EQ(run.pairs[i].estimate.timestamp(), times[i]);
        EXPECT_TRUE(run.pairs[i].estimate.positionWorldM().isApprox(expected_positions[i], 1e-13));
        EXPECT_TRUE(run.pairs[i].estimate.velocityWorldMps().isApprox(expected_velocities[i], 1e-13));
    }
}

TEST(EstimationEvaluationTest, EvaluatesTwoPosteriorSamplesAndReplaysExactly) {
    EvaluationRun first, replay;
    ASSERT_NO_FATAL_FAILURE(runEvaluation(first));
    ASSERT_NO_FATAL_FAILURE(runEvaluation(replay));
    ASSERT_EQ(first.pairs.size(), replay.pairs.size());
    for (std::size_t i = 0; i < first.pairs.size(); ++i) {
        EXPECT_EQ(first.pairs[i].estimate.vector(), replay.pairs[i].estimate.vector());
        EXPECT_EQ(first.pairs[i].estimate.covariance(), replay.pairs[i].estimate.covariance());
        EXPECT_EQ(first.pairs[i].estimate.timestamp(), replay.pairs[i].estimate.timestamp());
        EXPECT_EQ(first.pairs[i].truth.timestamp(), replay.pairs[i].truth.timestamp());
    }
    ASSERT_EQ(first.scores.size(), replay.scores.size());
    for (std::size_t i = 0; i < first.scores.size(); ++i) {
        EXPECT_EQ(first.scores[i].value, replay.scores[i].value);
        EXPECT_EQ(first.scores[i].degrees_of_freedom, replay.scores[i].degrees_of_freedom);
    }
    EXPECT_EQ(first.rmse->sample_count, replay.rmse->sample_count);
    EXPECT_EQ(first.rmse->position_axis_m, replay.rmse->position_axis_m);
    EXPECT_EQ(first.rmse->velocity_axis_mps, replay.rmse->velocity_axis_mps);
    EXPECT_EQ(first.rmse->position_3d_m, replay.rmse->position_3d_m);
    EXPECT_EQ(first.rmse->velocity_3d_mps, replay.rmse->velocity_3d_mps);
}

void expectError(const Score& result, EvaluationError error) {
    ASSERT_TRUE(std::holds_alternative<EvaluationError>(result));
    EXPECT_EQ(std::get<EvaluationError>(result), error);
}

TEST(EstimationEvaluationTest, ExposesAlignmentRankAndRangeFailures) {
    const SimulationTimestamp time{7, Time{1'000'000'000}};
    const SimulationTimestamp other_epoch{8, time.elapsed};
    const SimulationTimestamp later{7, time.elapsed + Time{1}};
    const auto prediction = KalmanPredictor::predict(
        initialEstimate(EstimatorCovariance6::Zero()), time, Matrix3::Zero());
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(prediction));
    const auto& prior = std::get<EstimatorState>(prediction);
    expectError(computeGpsNis(prior, gpsAt(other_epoch, Vector3::Zero(), 1)),
                EvaluationError::EpochMismatch);
    expectError(computeGpsNis(prior, gpsAt(later, Vector3::Zero(), 1)),
                EvaluationError::TimestampMismatch);
    expectError(computeGpsNis(prior, gpsAt(time, Vector3::Zero(), 0)),
                EvaluationError::UnsolvableCovariance);
    expectError(computeGpsNis(prior, gpsAt(time, Vector3(0x1p512, 0, 0), 1)),
                EvaluationError::NumericalFailure);

    const auto update = GpsKalmanUpdater::update(prior, gpsAt(time, Vector3::Zero(), 1));
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(update));
    const auto& posterior = std::get<EstimatorState>(update);
    expectError(computeNees(posterior, truthAt(time, Vector3::Zero()), NeesComponents::FullState),
                EvaluationError::UnsolvableCovariance);
    const std::array pairs{EstimateTruthPair{posterior, truthAt(time, Vector3::Zero())},
                           EstimateTruthPair{posterior, truthAt(later, Vector3::Zero())}};
    const auto result = computeRmse(pairs);
    ASSERT_TRUE(std::holds_alternative<RmseFailure>(result));
    EXPECT_EQ(std::get<RmseFailure>(result).error, EvaluationError::TimestampMismatch);
    EXPECT_EQ(std::get<RmseFailure>(result).sample_index, 1u);
}

} // namespace
} // namespace aerotwin
