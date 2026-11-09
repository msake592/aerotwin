#include <aerotwin/estimation/EstimatorState.hpp>

#include "../math/CovarianceValidation.hpp"

namespace aerotwin {

std::variant<EstimatorState, EstimatorStateError> EstimatorState::create(
    const EstimatorVector6& state, const EstimatorCovariance6& covariance,
    SimulationTimestamp timestamp) {
    if (!state.allFinite()) {
        return EstimatorStateError::InvalidState;
    }
    if (timestamp.elapsed < SimulationNanoseconds::zero()) {
        return EstimatorStateError::InvalidTimestamp;
    }

    Eigen::SelfAdjointEigenSolver<EstimatorCovariance6> solver;
    const auto symmetric = detail::validateCovariance(covariance, solver, Eigen::EigenvaluesOnly);
    if (!symmetric) {
        return EstimatorStateError::InvalidCovariance;
    }
    return EstimatorState{state, *symmetric, timestamp};
}

} // namespace aerotwin
