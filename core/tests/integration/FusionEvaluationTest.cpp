#include "FusionEvaluation.hpp"

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <utility>

namespace aerotwin::fusion_demo {
namespace {

const GpsMeasurement& observation(const FusionEvaluation& result, const GpsEvaluation& event) {
    return std::get<GpsMeasurement>(*result.acquisition.batches[event.tick_index][0]);
}

void expectSameState(const EstimatorState& a, const EstimatorState& b) {
    EXPECT_EQ(a.timestamp(), b.timestamp());
    EXPECT_EQ(a.vector(), b.vector());
    EXPECT_EQ(a.covariance(), b.covariance());
}

void expectSameScore(ConsistencyScore a, ConsistencyScore b) {
    EXPECT_EQ(a.value, b.value);
    EXPECT_EQ(a.degrees_of_freedom, b.degrees_of_freedom);
}

void expectSameEstimation(const FusionEvaluation& a, const FusionEvaluation& b) {
    ASSERT_EQ(a.gps_events.size(), b.gps_events.size());
    EXPECT_EQ(a.prediction_count, b.prediction_count);
    EXPECT_EQ(a.gps_update_count, b.gps_update_count);
    EXPECT_EQ(a.acquisition.gps_count, b.acquisition.gps_count);
    EXPECT_EQ(a.acquisition.imu_count, b.acquisition.imu_count);
    for (std::size_t i = 0; i < a.gps_events.size(); ++i) {
        const auto& x = a.gps_events[i];
        const auto& y = b.gps_events[i];
        EXPECT_EQ(x.tick_index, y.tick_index);
        const auto& gps_x = observation(a, x);
        const auto& gps_y = observation(b, y);
        EXPECT_EQ(gps_x.acquisition().timestamp(), gps_y.acquisition().timestamp());
        EXPECT_EQ(gps_x.acquisition().sensorId(), gps_y.acquisition().sensorId());
        EXPECT_EQ(gps_x.positionWorldM(), gps_y.positionWorldM());
        EXPECT_EQ(gps_x.positionCovarianceWorldM2(), gps_y.positionCovarianceWorldM2());
        expectSameState(x.prior, y.prior);
        expectSameState(x.posterior_truth.estimate, y.posterior_truth.estimate);
        expectSameScore(x.nis, y.nis);
    }
}

void expectSameTruthMetrics(const FusionEvaluation& a, const FusionEvaluation& b) {
    ASSERT_EQ(a.gps_events.size(), b.gps_events.size());
    for (std::size_t i = 0; i < a.gps_events.size(); ++i) {
        expectSameScore(a.gps_events[i].full_nees, b.gps_events[i].full_nees);
        expectSameScore(a.gps_events[i].position_nees, b.gps_events[i].position_nees);
        expectSameScore(a.gps_events[i].velocity_nees, b.gps_events[i].velocity_nees);
    }
    EXPECT_EQ(a.rmse.sample_count, b.rmse.sample_count);
    EXPECT_EQ(a.rmse.position_axis_m, b.rmse.position_axis_m);
    EXPECT_EQ(a.rmse.velocity_axis_mps, b.rmse.velocity_axis_mps);
    EXPECT_EQ(a.rmse.position_3d_m, b.rmse.position_3d_m);
    EXPECT_EQ(a.rmse.velocity_3d_mps, b.rmse.velocity_3d_mps);
}

void expectScore(ConsistencyScore score, unsigned dof) {
    EXPECT_TRUE(std::isfinite(score.value));
    EXPECT_GE(score.value, 0.0);
    EXPECT_EQ(score.degrees_of_freedom, dof);
}

TEST(FusionEvaluationTest, EvaluatesOnlyGpsEventsAtExactAcquisitionTimes) {
    auto acquired = acquireScenario();
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(acquired));
    const auto result = evaluateFusion(std::get<AcquisitionHistory>(std::move(acquired)));
    ASSERT_TRUE(std::holds_alternative<FusionEvaluation>(result));
    const auto& run = std::get<FusionEvaluation>(result);
    EXPECT_EQ(run.acquisition.scenario.samples().size(), 401u);
    EXPECT_EQ(run.acquisition.batches.size(), 401u);
    EXPECT_EQ(run.acquisition.gps_count, 5u);
    EXPECT_EQ(run.acquisition.imu_count, 401u);
    EXPECT_EQ(run.prediction_count, 4u);
    EXPECT_EQ(run.gps_update_count, 5u);
    EXPECT_EQ(run.gps_update_count, run.acquisition.gps_count);
    EXPECT_EQ(run.rmse.sample_count, 5u);
    ASSERT_EQ(run.gps_events.size(), 5u);
    const EstimatorVector6 diagonal = (EstimatorVector6() << 100, 100, 100, 4, 4, 4).finished();
    EXPECT_EQ(run.gps_events.front().prior.vector(), EstimatorVector6::Zero());
    EXPECT_EQ(run.gps_events.front().prior.covariance(), diagonal.asDiagonal().toDenseMatrix());

    Vector3 position_squared = Vector3::Zero();
    Vector3 velocity_squared = Vector3::Zero();
    for (std::size_t i = 0; i < run.gps_events.size(); ++i) {
        const auto& event = run.gps_events[i];
        const auto& pair = event.posterior_truth;
        const SimulationTimestamp time{0, std::chrono::seconds(i)};
        EXPECT_EQ(event.tick_index, i * 100);
        EXPECT_EQ(observation(run, event).acquisition().timestamp(), time);
        EXPECT_EQ(event.prior.timestamp(), time);
        EXPECT_EQ(pair.estimate.timestamp(), time);
        EXPECT_EQ(pair.truth.timestamp(), time);
        EXPECT_TRUE(event.prior.vector().allFinite());
        EXPECT_TRUE(event.prior.covariance().allFinite());
        EXPECT_TRUE(pair.estimate.vector().allFinite());
        EXPECT_TRUE(pair.estimate.covariance().allFinite());
        expectScore(event.nis, 3);
        expectScore(event.full_nees, 6);
        expectScore(event.position_nees, 3);
        expectScore(event.velocity_nees, 3);
        position_squared += (pair.estimate.positionWorldM() - pair.truth.positionWorldM()).cwiseAbs2();
        velocity_squared += (pair.estimate.velocityWorldMps() - pair.truth.linearVelocityWorldMps()).cwiseAbs2();
    }
    EXPECT_TRUE(run.rmse.position_axis_m.isApprox((position_squared / 5).cwiseSqrt(), 1e-13));
    EXPECT_TRUE(run.rmse.velocity_axis_mps.isApprox((velocity_squared / 5).cwiseSqrt(), 1e-13));
    EXPECT_NEAR(run.rmse.position_3d_m, std::sqrt(position_squared.sum() / 5), 1e-13);
    EXPECT_NEAR(run.rmse.velocity_3d_mps, std::sqrt(velocity_squared.sum() / 5), 1e-13);
}

TEST(FusionEvaluationTest, UsesPriorInnovationAndPredictsFromPreviousPosterior) {
    auto acquired = acquireScenario();
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(acquired));
    const auto result = evaluateFusion(std::get<AcquisitionHistory>(std::move(acquired)));
    ASSERT_TRUE(std::holds_alternative<FusionEvaluation>(result));
    const auto& run = std::get<FusionEvaluation>(result);
    EstimatorCovariance6 transition = EstimatorCovariance6::Identity();
    transition.topRightCorner<3, 3>().setIdentity(); // One second between GPS acquisitions.
    const auto identity = Eigen::Matrix3d::Identity();
    EstimatorCovariance6 process_noise;
    process_noise.topLeftCorner<3, 3>() = (0.01 / 3) * identity;
    process_noise.topRightCorner<3, 3>() = 0.005 * identity;
    process_noise.bottomLeftCorner<3, 3>() = 0.005 * identity;
    process_noise.bottomRightCorner<3, 3>() = 0.01 * identity;

    for (std::size_t i = 0; i < run.gps_events.size(); ++i) {
        const auto& event = run.gps_events[i];
        const auto& gps = observation(run, event);
        const Vector3 innovation = gps.positionWorldM() - event.prior.positionWorldM();
        const Eigen::Matrix3d innovation_covariance =
            event.prior.covariance().topLeftCorner<3, 3>() + gps.positionCovarianceWorldM2();
        const auto solver = innovation_covariance.llt();
        ASSERT_EQ(solver.info(), Eigen::Success);
        EXPECT_NEAR(event.nis.value, innovation.dot(solver.solve(innovation)), 1e-13);
        const EstimatorCovariance6 prior_covariance = event.prior.covariance();
        const Eigen::Matrix<double, 6, 3> gain =
            solver.solve(prior_covariance.topRows<3>()).transpose();
        const auto& posterior = event.posterior_truth.estimate;
        EXPECT_TRUE(posterior.vector().isApprox(event.prior.vector() + gain * innovation, 1e-13));
        EXPECT_TRUE(posterior.covariance().isApprox(
            prior_covariance - gain * prior_covariance.topRows<3>(), 1e-13));

        if (i > 0) {
            const auto& previous = run.gps_events[i - 1].posterior_truth.estimate;
            EXPECT_LT(previous.timestamp().elapsed, event.prior.timestamp().elapsed);
            EXPECT_TRUE(event.prior.vector().isApprox(transition * previous.vector(), 1e-13));
            EXPECT_TRUE(event.prior.covariance().isApprox(
                transition * previous.covariance() * transition.transpose() + process_noise, 1e-13));
        }
    }
}

TEST(FusionEvaluationTest, AlternateTruthChangesOnlyTruthBasedScores) {
    auto a = acquireScenario();
    auto b = acquireScenario();
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(a));
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(b));
    auto& comparison = std::get<AcquisitionHistory>(b);
    // Feed precisely the original sampled observations to both runners.
    comparison.batches = std::get<AcquisitionHistory>(a).batches;
    std::vector<VehicleState> alternate_truth;
    for (const auto& truth : comparison.scenario.samples()) {
        const auto shifted = VehicleState::create(
            truth.positionWorldM() + Vector3(20, 0, 0), truth.orientationWorldFromBody(),
            truth.linearVelocityWorldMps(), truth.angularVelocityBodyRadps(), truth.timestamp());
        ASSERT_TRUE(shifted);
        alternate_truth.push_back(*shifted);
    }

    const auto first = evaluateFusion(std::get<AcquisitionHistory>(std::move(a)));
    const auto second = evaluateFusion(std::move(comparison), alternate_truth);
    ASSERT_TRUE(std::holds_alternative<FusionEvaluation>(first));
    ASSERT_TRUE(std::holds_alternative<FusionEvaluation>(second));
    const auto& original = std::get<FusionEvaluation>(first);
    const auto& changed = std::get<FusionEvaluation>(second);
    expectSameEstimation(original, changed);
    EXPECT_NE(original.rmse.position_3d_m, changed.rmse.position_3d_m);
    EXPECT_NE(original.gps_events.front().full_nees.value, changed.gps_events.front().full_nees.value);
}

TEST(FusionEvaluationTest, FreshRunsReplayExactlyAndImuSeedsAffectOnlyDiagnostics) {
    auto first = acquireScenario();
    auto replay = acquireScenario();
    auto alternate_imu = acquireScenario(std::chrono::seconds(1), {45, 46});
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(first));
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(replay));
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(alternate_imu));
    const auto a = evaluateFusion(std::get<AcquisitionHistory>(std::move(first)));
    const auto b = evaluateFusion(std::get<AcquisitionHistory>(std::move(replay)));
    const auto c = evaluateFusion(std::get<AcquisitionHistory>(std::move(alternate_imu)));
    ASSERT_TRUE(std::holds_alternative<FusionEvaluation>(a));
    ASSERT_TRUE(std::holds_alternative<FusionEvaluation>(b));
    ASSERT_TRUE(std::holds_alternative<FusionEvaluation>(c));
    const auto& base = std::get<FusionEvaluation>(a);
    const auto& repeated = std::get<FusionEvaluation>(b);
    const auto& changed = std::get<FusionEvaluation>(c);
    expectSameEstimation(base, repeated);
    expectSameTruthMetrics(base, repeated);
    expectSameEstimation(base, changed);
    expectSameTruthMetrics(base, changed);
    const auto& imu = std::get<ImuMeasurement>(*base.acquisition.batches.front()[1]);
    const auto& other = std::get<ImuMeasurement>(*changed.acquisition.batches.front()[1]);
    EXPECT_EQ(imu.acquisition().timestamp(), other.acquisition().timestamp());
    EXPECT_EQ(imu.acquisition().sensorId(), other.acquisition().sensorId());
    EXPECT_NE(imu.specificForceBodyMps2(), other.specificForceBodyMps2());
    EXPECT_NE(imu.angularVelocityBodyRadps(), other.angularVelocityBodyRadps());
}

TEST(FusionEvaluationTest, PreservesTimestampAndEpochMismatchContext) {
    for (const auto& [time, error] :
         std::array{std::pair{SimulationTimestamp{0, SimulationNanoseconds{1}},
                              EvaluationError::TimestampMismatch},
                    std::pair{SimulationTimestamp{1, {}}, EvaluationError::EpochMismatch}}) {
        auto acquired = acquireScenario();
        ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(acquired));
        auto& history = std::get<AcquisitionHistory>(acquired);
        auto& slot = history.batches.front()[0];
        const auto& gps = std::get<GpsMeasurement>(*slot);
        const auto metadata = MeasurementAcquisition::create(gps.acquisition().sensorId(), time);
        ASSERT_TRUE(metadata);
        const auto replacement = GpsMeasurement::create(
            *metadata, gps.positionWorldM(), gps.positionCovarianceWorldM2());
        ASSERT_TRUE(replacement);
        slot = *replacement;
        const auto result = evaluateFusion(std::move(history));
        ASSERT_TRUE(std::holds_alternative<FusionFailure>(result));
        const auto& failure = std::get<FusionFailure>(result);
        EXPECT_EQ(failure.stage, FusionStage::AcquisitionAlignment);
        EXPECT_EQ(failure.timestamp, time);
        ASSERT_TRUE(failure.observation);
        EXPECT_EQ(failure.observation->sensorId(), "gps");
        EXPECT_EQ(failure.error, FusionError(error));
    }
}

TEST(FusionEvaluationTest, PreservesSingularInnovationAndPosteriorEvaluationFailures) {
    for (bool singular_prior : {false, true}) {
        auto acquired = acquireScenario();
        ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(acquired));
        auto& history = std::get<AcquisitionHistory>(acquired);
        auto& slot = history.batches.front()[0];
        const auto& gps = std::get<GpsMeasurement>(*slot);
        const auto exact = GpsMeasurement::create(
            gps.acquisition(), gps.positionWorldM(), Eigen::Matrix3d::Zero());
        ASSERT_TRUE(exact);
        slot = *exact;
        std::optional<EstimatorCovariance6> covariance;
        if (singular_prior) {
            covariance = EstimatorCovariance6::Zero();
        }
        const auto result = evaluateFusion(std::move(history), {}, covariance);
        ASSERT_TRUE(std::holds_alternative<FusionFailure>(result));
        const auto& failure = std::get<FusionFailure>(result);
        EXPECT_EQ(failure.stage, singular_prior ? FusionStage::GpsNis : FusionStage::Nees);
        EXPECT_EQ(failure.timestamp, SimulationTimestamp{});
        ASSERT_TRUE(failure.observation);
        EXPECT_EQ(failure.observation->sensorId(), "gps");
        EXPECT_EQ(failure.error, FusionError(EvaluationError::UnsolvableCovariance));
    }
}

TEST(FusionEvaluationTest, RejectsSwappedSlotsAndInconsistentCounts) {
    for (bool swap_slots : {false, true}) {
        auto acquired = acquireScenario();
        ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(acquired));
        auto& history = std::get<AcquisitionHistory>(acquired);
        if (swap_slots) {
            std::swap(history.batches.front()[0], history.batches.front()[1]);
        } else {
            ++history.gps_count;
        }
        const auto result = evaluateFusion(std::move(history));
        ASSERT_TRUE(std::holds_alternative<FusionFailure>(result));
        const auto& failure = std::get<FusionFailure>(result);
        EXPECT_EQ(failure.stage, FusionStage::AcquisitionHistory);
        EXPECT_EQ(failure.timestamp, (SimulationTimestamp{0, std::chrono::seconds(swap_slots ? 0 : 4)}));
    }
}

TEST(FusionEvaluationTest, PreservesInvalidInitializationError) {
    auto acquired = acquireScenario();
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(acquired));
    const auto result = evaluateFusion(std::get<AcquisitionHistory>(std::move(acquired)), {},
                                       -EstimatorCovariance6::Identity());
    ASSERT_TRUE(std::holds_alternative<FusionFailure>(result));
    const auto& failure = std::get<FusionFailure>(result);
    EXPECT_EQ(failure.stage, FusionStage::Initialization);
    EXPECT_EQ(failure.timestamp, SimulationTimestamp{});
    EXPECT_EQ(failure.error, FusionError(EstimatorStateError::InvalidCovariance));
}

} // namespace
} // namespace aerotwin::fusion_demo
