#pragma once

#include <aerotwin/estimation/EstimatorState.hpp>

namespace aerotwin {

enum class PredictionError {
    InvalidProcessNoise,
    InvalidTargetTimestamp,
    EpochMismatch,
    NonForwardTime,
    NumericalOverflow,
    InvalidPropagatedCovariance,
};

using PredictionResult = std::variant<EstimatorState, PredictionError>;

class KalmanPredictor {
public:
    // Continuous white-acceleration spectral density in world ENU, in m²/s³.
    // The target must be strictly later in the current state's epoch.
    [[nodiscard]] static PredictionResult predict(
        const EstimatorState& current, SimulationTimestamp target,
        const Eigen::Matrix3d& acceleration_spectral_density_world_m2ps3);
};

} // namespace aerotwin
