#pragma once

#include <aerotwin/estimation/EstimatorState.hpp>
#include <aerotwin/sensors/Measurements.hpp>

namespace aerotwin {

enum class GpsUpdateError {
    EpochMismatch,
    TimestampMismatch,
    InvalidInnovationCovariance,
    UnsolvableInnovationCovariance,
    NumericalFailure,
    InvalidPosteriorCovariance,
};

using GpsUpdateResult = std::variant<EstimatorState, GpsUpdateError>;

class GpsKalmanUpdater {
public:
    // Corrects an estimate with a validated ENU position observation at the exact same instant.
    [[nodiscard]] static GpsUpdateResult update(const EstimatorState& prior,
                                                const GpsMeasurement& measurement);
};

} // namespace aerotwin
