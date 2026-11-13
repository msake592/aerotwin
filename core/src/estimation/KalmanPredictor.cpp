#include <aerotwin/estimation/KalmanPredictor.hpp>

#include "../math/CovarianceValidation.hpp"

#include <algorithm>
#include <cmath>

namespace aerotwin {
namespace {

// Keep small sums away from the subnormal range until the final result.
template <typename Evaluate, typename... Coefficients>
double evaluateCovarianceEntry(Evaluate evaluate, Coefficients... coefficients) {
    const double magnitude = std::max({std::abs(coefficients)...});
    // A nanosecond step can shrink a coefficient by roughly 2^-60 after
    // two dt factors. Include nearby normal values as well as subnormals.
    const bool has_tiny_coefficient =
        ((coefficients != 0.0 && std::abs(coefficients) < 0x1p-950) || ...);
    // The largest weight is below 2^100. After scaling, each weighted
    // term is below 2^1018, leaving room for the sum of five terms.
    if (has_tiny_coefficient && magnitude <= 0x1p790) {
        constexpr double scale = 0x1p128;
        return evaluate((coefficients * scale)...) / scale;
    }
    return evaluate(coefficients...);
}

// Evaluate F P Fᵀ + Q using the nonzero blocks of F. Compute each symmetric
// entry once. Group the position terms by transition row to preserve
// cancellation before adding process noise.
EstimatorCovariance6 propagate(const EstimatorCovariance6& covariance,
                               const Eigen::Matrix3d& density, double dt) {
    const double dt2 = dt * dt;
    const double cross_weight = dt2 / 2.0;
    const double position_weight = dt2 * dt / 3.0;
    EstimatorCovariance6 result;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            const double vv = covariance(row + 3, column + 3);
            const double noise = density(row, column);
            const double pv = covariance(row, column + 3);
            const double cross = evaluateCovarianceEntry(
                [dt, cross_weight](double cross_covariance, double velocity, double spectral) {
                    return std::fma(cross_weight, spectral, std::fma(dt, velocity, cross_covariance));
                },
                pv, vv, noise);
            result(row, column + 3) = result(column + 3, row) = cross;
            if (column <= row) {
                const double vp = covariance(row + 3, column);
                const double position = evaluateCovarianceEntry(
                    [dt, position_weight](double pp, double pv, double vp, double vv,
                                          double spectral) {
                        const double left = std::fma(dt, pv, pp);
                        const double right = std::fma(dt, vv, vp);
                        return std::fma(position_weight, spectral, std::fma(dt, right, left));
                    },
                    covariance(row, column), pv, vp, vv, noise);
                result(row, column) = result(column, row) = position;
                const double velocity = std::fma(dt, noise, vv);
                result(row + 3, column + 3) = result(column + 3, row + 3) = velocity;
            }
        }
    }
    return result;
}

} // namespace

PredictionResult KalmanPredictor::predict(const EstimatorState& current,
                                          SimulationTimestamp target,
                                          const Eigen::Matrix3d& density) {
    const auto start = current.timestamp();
    if (target.elapsed < SimulationNanoseconds::zero()) {
        return PredictionError::InvalidTargetTimestamp;
    }
    if (target.epoch != start.epoch) {
        return PredictionError::EpochMismatch;
    }
    if (target.elapsed <= start.elapsed) {
        return PredictionError::NonForwardTime;
    }

    // Validated operands satisfy 0 <= start < target <= int64 max.
    const auto elapsed = target.elapsed - start.elapsed;
    const double dt = std::chrono::duration<double>(elapsed).count();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    const auto symmetric_density = detail::validateCovariance(density, solver, Eigen::EigenvaluesOnly);
    if (!symmetric_density) {
        return PredictionError::InvalidProcessNoise;
    }

    EstimatorCovariance6 transition = EstimatorCovariance6::Identity();
    transition.topRightCorner<3, 3>().diagonal().setConstant(dt);
    EstimatorVector6 state = current.vector();
    // Sparse F * x, with a fused multiply-add to avoid overflow before cancellation.
    for (int axis = 0; axis < 3; ++axis) {
        state(axis) = std::fma(transition(axis, axis + 3), state(axis + 3), state(axis));
    }
    if (!state.allFinite()) {
        return PredictionError::NumericalOverflow;
    }

    const auto covariance = current.covariance();
    auto propagated = propagate(covariance, *symmetric_density, dt);
    if (!propagated.allFinite()) {
        // The largest cubic weight is below 2^100 for signed 64-bit nanoseconds.
        // Retry with headroom for every weighted term and their sum.
        constexpr double scale = 0x1p-128;
        const EstimatorCovariance6 retry =
            propagate(covariance * scale, *symmetric_density * scale, dt) / scale;
        for (int i = 0; i < propagated.size(); ++i) {
            if (!std::isfinite(propagated(i))) {
                propagated(i) = retry(i);
            }
        }
        if (!propagated.allFinite()) {
            return PredictionError::NumericalOverflow;
        }
    }

    auto result = EstimatorState::create(state, propagated, target);
    if (const auto* predicted = std::get_if<EstimatorState>(&result)) {
        return *predicted;
    }
    // State finiteness and target validity were checked above.
    return PredictionError::InvalidPropagatedCovariance;
}

} // namespace aerotwin
