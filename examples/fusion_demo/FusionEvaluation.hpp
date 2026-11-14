#pragma once

#include "ScenarioAcquisition.hpp"

#include <aerotwin/estimation/GpsKalmanUpdater.hpp>
#include <aerotwin/estimation/KalmanPredictor.hpp>
#include <aerotwin/validation/EstimationMetrics.hpp>

namespace aerotwin::fusion_demo {

inline constexpr double kAccelerationSpectralDensityM2ps3 = 0.01;

struct GpsEvaluation {
    // The original GPS observation is in acquisition.batches[tick_index][0].
    std::size_t tick_index;
    EstimatorState prior;
    ConsistencyScore nis;
    EstimateTruthPair posterior_truth;
    ConsistencyScore full_nees;
    ConsistencyScore position_nees;
    ConsistencyScore velocity_nees;
};

struct FusionEvaluation {
    AcquisitionHistory acquisition;
    std::vector<GpsEvaluation> gps_events;
    RmseResult rmse;
    std::size_t prediction_count;
    std::size_t gps_update_count;
};

enum class FusionStage {
    AcquisitionHistory,
    AcquisitionAlignment,
    Initialization,
    Prediction,
    GpsNis,
    GpsCorrection,
    Nees,
    Rmse,
};

using FusionError = std::variant<std::monostate, EstimatorStateError, PredictionError,
                                 GpsUpdateError, EvaluationError>;

struct FusionFailure {
    FusionStage stage;
    SimulationTimestamp timestamp{};
    std::optional<MeasurementAcquisition> observation{};
    FusionError error{};
    std::optional<std::size_t> sample_index{};
};

using FusionResult = std::variant<FusionEvaluation, FusionFailure>;

// Alternate evaluation truth is optional and indexed by the original tick.
// The initial covariance override supports scenarios with different prior uncertainty.
[[nodiscard]] FusionResult evaluateFusion(
    AcquisitionHistory history, std::span<const VehicleState> evaluation_truth = {},
    std::optional<EstimatorCovariance6> initial_covariance = std::nullopt);

} // namespace aerotwin::fusion_demo
