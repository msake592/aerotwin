#pragma once

#include <aerotwin/math/Types.hpp>
#include <aerotwin/simulation/SimulationClock.hpp>

#include <variant>

namespace aerotwin {

// World ENU, ordered [px, py, pz, vx, vy, vz] in metres and metres per second.
using EstimatorVector6 = Eigen::Matrix<double, 6, 1>;
// Entry (i, j) has units equal to the product of state component units.
using EstimatorCovariance6 = Eigen::Matrix<double, 6, 6>;

enum class EstimatorStateError {
    InvalidState,
    InvalidTimestamp,
    InvalidCovariance,
};

// Owned, validated estimate at a precise simulation instant.
class EstimatorState {
public:
    [[nodiscard]] static std::variant<EstimatorState, EstimatorStateError> create(
        const EstimatorVector6& state, const EstimatorCovariance6& covariance,
        SimulationTimestamp timestamp);

    [[nodiscard]] EstimatorVector6 vector() const noexcept { return state_; }
    [[nodiscard]] Vector3 positionWorldM() const noexcept { return state_.head<3>(); }
    [[nodiscard]] Vector3 velocityWorldMps() const noexcept { return state_.tail<3>(); }
    [[nodiscard]] EstimatorCovariance6 covariance() const noexcept { return covariance_; }
    [[nodiscard]] SimulationTimestamp timestamp() const noexcept { return timestamp_; }

private:
    EstimatorState(const EstimatorVector6& state, const EstimatorCovariance6& covariance,
                   SimulationTimestamp timestamp) noexcept
        : state_(state), covariance_(covariance), timestamp_(timestamp) {}

    EstimatorVector6 state_;
    EstimatorCovariance6 covariance_;
    SimulationTimestamp timestamp_;
};

} // namespace aerotwin
