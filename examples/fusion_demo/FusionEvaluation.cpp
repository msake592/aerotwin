#include "FusionEvaluation.hpp"

#include <utility>

namespace aerotwin::fusion_demo {
namespace {

struct GpsEstimate {
    EstimatorState prior;
    ConsistencyScore nis;
    EstimatorState posterior;
};

std::variant<GpsEstimate, FusionFailure> estimateGps(const EstimatorState& previous,
                                                  const GpsMeasurement& gps, bool predict) {
    const auto& acquisition = gps.acquisition();
    const auto timestamp = acquisition.timestamp();
    EstimatorState prior = previous;
    if (predict) {
        const auto prediction =
            KalmanPredictor::predict(previous, timestamp, kAccelerationSpectralDensityM2ps3 * Eigen::Matrix3d::Identity());
        if (const auto* error = std::get_if<PredictionError>(&prediction)) {
            return FusionFailure{FusionStage::Prediction, timestamp, acquisition, *error};
        }
        prior = std::get<EstimatorState>(prediction);
    }

    const auto nis = computeGpsNis(prior, gps);
    if (const auto* error = std::get_if<EvaluationError>(&nis)) {
        return FusionFailure{FusionStage::GpsNis, timestamp, acquisition, *error};
    }
    const auto update = GpsKalmanUpdater::update(prior, gps);
    if (const auto* error = std::get_if<GpsUpdateError>(&update)) {
        return FusionFailure{FusionStage::GpsCorrection, timestamp, acquisition, *error};
    }
    return GpsEstimate{prior, std::get<ConsistencyScore>(nis), std::get<EstimatorState>(update)};
}

std::optional<FusionFailure> checkAlignment(const MeasurementAcquisition& observation,
                                            SimulationTimestamp tick) {
    const auto timestamp = observation.timestamp();
    if (timestamp != tick) {
        const auto error = timestamp.epoch != tick.epoch ? EvaluationError::EpochMismatch
                                                         : EvaluationError::TimestampMismatch;
        return FusionFailure{FusionStage::AcquisitionAlignment, timestamp, observation, error};
    }
    return std::nullopt;
}

} // namespace

FusionResult evaluateFusion(AcquisitionHistory history, std::span<const VehicleState> evaluation_truth,
                            std::optional<EstimatorCovariance6> initial_covariance) {
    const auto& ticks = history.scenario.samples();
    if (ticks.empty() || ticks.size() != history.batches.size() ||
        (!evaluation_truth.empty() && evaluation_truth.size() != ticks.size())) {
        return FusionFailure{FusionStage::AcquisitionHistory};
    }
    if (evaluation_truth.empty()) {
        evaluation_truth = ticks;
    }
    if (!initial_covariance) {
        initial_covariance = (EstimatorVector6() << 100, 100, 100, 4, 4, 4).finished().asDiagonal();
    }
    const auto initial = EstimatorState::create(EstimatorVector6::Zero(), *initial_covariance, {});
    if (const auto* error = std::get_if<EstimatorStateError>(&initial)) {
        return FusionFailure{FusionStage::Initialization, {}, {}, *error};
    }
    EstimatorState previous = std::get<EstimatorState>(initial);
    std::vector<GpsEvaluation> events;
    std::vector<EstimateTruthPair> pairs;
    std::size_t prediction_count = 0;
    std::size_t imu_count = 0;

    for (std::size_t i = 0; i < ticks.size(); ++i) {
        const auto tick = ticks[i].timestamp();
        const auto& batch = history.batches[i];
        const auto* gps = batch[0] ? std::get_if<GpsMeasurement>(&*batch[0]) : nullptr;
        const auto& imu_slot = batch[gps ? 1 : 0];
        const auto* imu = imu_slot ? std::get_if<ImuMeasurement>(&*imu_slot) : nullptr;
        if (!imu || (!gps && (i == 0 || batch[1]))) {
            return FusionFailure{FusionStage::AcquisitionHistory, tick};
        }
        if (gps) {
            if (const auto failure = checkAlignment(gps->acquisition(), tick)) {
                return *failure;
            }
            auto result = estimateGps(previous, *gps, !events.empty());
            if (const auto* failure = std::get_if<FusionFailure>(&result)) {
                return *failure;
            }
            const auto& estimated = std::get<GpsEstimate>(result);
            const auto& truth = evaluation_truth[i];
            constexpr std::array components{
                NeesComponents::FullState, NeesComponents::Position, NeesComponents::Velocity};
            std::array<ConsistencyScore, components.size()> scores{};
            for (std::size_t j = 0; j < components.size(); ++j) {
                const auto score = computeNees(estimated.posterior, truth, components[j]);
                if (const auto* error = std::get_if<EvaluationError>(&score)) {
                    return FusionFailure{FusionStage::Nees, tick, gps->acquisition(), *error};
                }
                scores[j] = std::get<ConsistencyScore>(score);
            }

            prediction_count += !events.empty();
            previous = estimated.posterior;
            pairs.push_back({estimated.posterior, truth});
            events.push_back({i, estimated.prior, estimated.nis, pairs.back(),
                              scores[0], scores[1], scores[2]});
        }
        if (const auto failure = checkAlignment(imu->acquisition(), tick)) {
            return *failure;
        }
        ++imu_count;
    }
    if (events.size() != history.gps_count || imu_count != history.imu_count) {
        return FusionFailure{FusionStage::AcquisitionHistory, ticks.back().timestamp()};
    }

    const auto rmse = computeRmse(pairs);
    if (const auto* failure = std::get_if<RmseFailure>(&rmse)) {
        FusionFailure context{FusionStage::Rmse, ticks.back().timestamp(), {}, failure->error,
                              failure->sample_index};
        if (failure->sample_index) {
            const auto& observation = std::get<GpsMeasurement>(
                *history.batches[events[*failure->sample_index].tick_index][0]);
            context.timestamp = observation.acquisition().timestamp();
            context.observation = observation.acquisition();
        }
        return context;
    }
    const auto update_count = events.size();
    return FusionEvaluation{std::move(history), std::move(events), std::get<RmseResult>(rmse),
                            prediction_count, update_count};
}

} // namespace aerotwin::fusion_demo
