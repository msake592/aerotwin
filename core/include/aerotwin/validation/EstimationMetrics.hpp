#pragma once

#include <aerotwin/estimation/EstimatorState.hpp>
#include <aerotwin/sensors/Measurements.hpp>
#include <aerotwin/simulation/VehicleState.hpp>

#include <cstddef>
#include <optional>
#include <span>
#include <variant>

namespace aerotwin {

struct EstimateTruthPair {
    EstimatorState estimate;
    VehicleState truth;
};

struct RmseResult {
    std::size_t sample_count;
    Vector3 position_axis_m;
    Vector3 velocity_axis_mps;
    double position_3d_m;
    double velocity_3d_mps;
};

enum class EvaluationError {
    EmptySamples,
    EpochMismatch,
    TimestampMismatch,
    NumericalFailure,
    InvalidCovariance,
    UnsolvableCovariance,
};

struct RmseFailure {
    EvaluationError error;
    // Zero-based index for pair-level failures. Batch failures have no index.
    std::optional<std::size_t> sample_index;
};

// Evaluate equal-weight ENU errors from one run and epoch. Snapshots in each
// pair must have matching timestamps. The caller selects the estimator phase.
[[nodiscard]] std::variant<RmseResult, RmseFailure>
computeRmse(std::span<const EstimateTruthPair> samples);

enum class NeesComponents { FullState, Position, Velocity };

struct ConsistencyScore {
    double value;
    unsigned degrees_of_freedom;
};

// Dimensionless score for an exact-time estimate and truth pair from the same run.
[[nodiscard]] std::variant<ConsistencyScore, EvaluationError>
computeNees(const EstimatorState& estimate, const VehicleState& truth, NeesComponents components);

// Dimensionless GPS innovation score for the prior at the acquisition timestamp.
[[nodiscard]] std::variant<ConsistencyScore, EvaluationError>
computeGpsNis(const EstimatorState& prior, const GpsMeasurement& measurement);

} // namespace aerotwin
