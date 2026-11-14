#include <aerotwin/validation/EstimationMetrics.hpp>

#include "../math/CovarianceValidation.hpp"

#include <Eigen/Cholesky>

#include <array>
#include <cmath>
#include <limits>

namespace aerotwin {
namespace {

struct BinaryValue {
    double fraction;
    int exponent;
};

// Halve large opposite-sign operands before subtracting, then restore scale
// in the exponent without materializing a possibly oversized difference.
BinaryValue difference(double estimate, double truth) {
    constexpr double half_max = std::numeric_limits<double>::max() * 0.5;
    const bool halve = std::signbit(estimate) != std::signbit(truth) &&
        (std::abs(estimate) > half_max || std::abs(truth) > half_max);
    const double residual = halve ? estimate * 0.5 - truth * 0.5 : estimate - truth;
    int exponent;
    const double fraction = std::frexp(residual, &exponent);
    return {fraction, exponent + (halve ? 1 : 0)};
}

// Store the sum of squares in units of 2^(2 * exponent_).
class ScaledSquares {
public:
    void add(double fraction, int exponent) {
        if (fraction == 0.0) {
            return;
        }
        if (sum_ == 0.0) {
            exponent_ = exponent;
        } else if (exponent > exponent_) {
            sum_ = std::scalbn(sum_, 2 * (exponent_ - exponent));
            exponent_ = exponent;
        }
        const double normalized = std::scalbn(fraction, exponent - exponent_);
        sum_ = std::fma(normalized, normalized, sum_);
    }

    std::optional<double> rootMeanSquare(std::size_t count) const {
        if (sum_ == 0.0) {
            return 0.0;
        }
        const double result = std::scalbn(std::sqrt(sum_ / static_cast<double>(count)), exponent_);
        if (!std::isfinite(result) || result == 0.0) {
            return std::nullopt;
        }
        return result;
    }

private:
    int exponent_{};
    double sum_{};
};

class VectorSquares {
public:
    void add(const Vector3& estimate, const Vector3& truth) {
        for (std::size_t axis = 0; axis < axes_.size(); ++axis) {
            const auto [fraction, exponent] = difference(estimate(axis), truth(axis));
            axes_[axis].add(fraction, exponent);
            spatial_.add(fraction, exponent);
        }
    }

    bool finish(std::size_t count, Vector3& axes, double& spatial) const {
        const auto norm = spatial_.rootMeanSquare(count);
        if (!norm) {
            return false;
        }
        spatial = *norm;
        for (std::size_t axis = 0; axis < axes_.size(); ++axis) {
            const auto value = axes_[axis].rootMeanSquare(count);
            if (!value) {
                return false;
            }
            axes(axis) = *value;
        }
        return true;
    }

private:
    std::array<ScaledSquares, 3> axes_;
    ScaledSquares spatial_;
};

using ScoreResult = std::variant<ConsistencyScore, EvaluationError>;

// Round on the subnormal grid while the accumulator still has two parts.
double restoreScore(double sum, double correction, int exponent) {
    constexpr int normal_exponent = std::numeric_limits<double>::min_exponent - 1;
    if (std::ilogb(sum) + exponent > normal_exponent) {
        return std::scalbn(sum + correction, exponent);
    }

    constexpr int unit_exponent =
        std::numeric_limits<double>::min_exponent - std::numeric_limits<double>::digits;
    const int shift = exponent - unit_exponent;
    const double high = std::scalbn(sum, shift);
    const double low = std::scalbn(correction, shift);

    // Keep whole units out of the fractional sum. FastTwoSum recovers
    // the fractional addition remainder under round-to-nearest-even.
    double high_integer, low_integer;
    double a = std::modf(high, &high_integer);
    double b = std::modf(low, &low_integer);
    if (std::abs(a) < std::abs(b)) {
        std::swap(a, b);
    }
    const double fraction = a + b;
    const double remainder = b - (fraction - a);
    const double lower = std::floor(fraction);
    const double midpoint = lower + 0.5;
    double units = high_integer + low_integer + lower;
    const double alignment_remainder = correction - std::scalbn(low, -shift);
    const double tie_remainder = remainder == 0.0 ? alignment_remainder : remainder;
    const bool round_tie_up =
        tie_remainder > 0.0 || (tie_remainder == 0.0 && std::fmod(units, 2.0) != 0.0);
    if (fraction > midpoint || (fraction == midpoint && round_tie_up)) {
        units += 1.0;
    }
    return std::scalbn(units, unit_exponent);
}

// The residual components are finite fractions with separate binary exponents.
// Reject a normalized RHS that loses normal-range precision before whitening.
template <int Dimension>
ScoreResult whitenedScore(const Eigen::Matrix<double, Dimension, Dimension>& covariance,
                          const std::array<BinaryValue, Dimension>& residual) {
    using Matrix = Eigen::Matrix<double, Dimension, Dimension>;
    using Vector = Eigen::Matrix<double, Dimension, 1>;
    Eigen::SelfAdjointEigenSolver<Matrix> spectrum;
    const auto symmetric = detail::validateCovariance(covariance, spectrum, Eigen::EigenvaluesOnly);
    if (!symmetric) {
        return EvaluationError::InvalidCovariance;
    }
    const Vector variances = symmetric->diagonal();
    constexpr double relative_floor = Dimension * std::numeric_limits<double>::epsilon();
    if ((variances.array() == 0.0).any() ||
        spectrum.eigenvalues().minCoeff() <= relative_floor * spectrum.eigenvalues().maxCoeff()) {
        return EvaluationError::UnsolvableCovariance;
    }

    // Power-of-two equilibration leaves the variance in the LDLᵀ pivots
    // instead of folding it into a rounded standard deviation.
    std::array<int, Dimension> scale_exponents;
    for (int axis = 0; axis < Dimension; ++axis) {
        int variance_exponent;
        std::frexp(variances(axis), &variance_exponent);
        scale_exponents[axis] = variance_exponent / 2;
    }
    Matrix equilibrated;
    for (int row = 0; row < Dimension; ++row) {
        for (int column = 0; column <= row; ++column) {
            equilibrated(row, column) = equilibrated(column, row) =
                std::scalbn((*symmetric)(row, column),
                            -scale_exponents[row] - scale_exponents[column]);
        }
    }
    const Eigen::LDLT<Matrix> factor(equilibrated);
    const Vector pivots = factor.vectorD();
    if (factor.info() != Eigen::Success || !pivots.allFinite() ||
        (pivots.array() <= 0.0).any()) {
        return EvaluationError::UnsolvableCovariance;
    }

    std::array<BinaryValue, Dimension> normalized;
    int common_exponent = std::numeric_limits<int>::min();
    for (int axis = 0; axis < Dimension; ++axis) {
        const auto [fraction, residual_exponent] = residual[axis];
        const int exponent = residual_exponent - scale_exponents[axis];
        normalized[axis] = {fraction, exponent};
        if (fraction != 0.0) {
            common_exponent = std::max(common_exponent, exponent);
        }
    }
    if (common_exponent == std::numeric_limits<int>::min()) {
        return ConsistencyScore{0.0, Dimension};
    }

    Vector rhs;
    for (int axis = 0; axis < Dimension; ++axis) {
        const auto [fraction, exponent] = normalized[axis];
        rhs(axis) = std::scalbn(fraction, exponent - common_exponent);
        if (fraction != 0.0 && std::abs(rhs(axis)) < std::numeric_limits<double>::min()) {
            return EvaluationError::NumericalFailure;
        }
    }
    // Eigen factors P A Pᵀ = L D Lᵀ. Apply P before the forward solve.
    const Vector whitened = factor.matrixL().solve(factor.transpositionsP() * rhs);
    if (!whitened.allFinite()) {
        return EvaluationError::NumericalFailure;
    }

    struct Contribution {
        double high;
        double low;
        int exponent;
    };
    std::array<Contribution, Dimension> terms;
    int sum_exponent = std::numeric_limits<int>::min();
    for (int axis = 0; axis < Dimension; ++axis) {
        int value_exponent;
        const double value = std::frexp(whitened(axis), &value_exponent);
        int pivot_exponent;
        const double pivot = std::frexp(pivots(axis), &pivot_exponent);
        // Nonzero frexp fractions have magnitude in [0.5, 1), so the
        // square and its FMA remainder are safely within the normal range.
        const double square = value * value;
        const double square_error = std::fma(value, value, -square);
        const double quotient = square / pivot;
        const double division_error = std::fma(-quotient, pivot, square);
        terms[axis] = {quotient, (division_error + square_error) / pivot,
                       2 * value_exponent - pivot_exponent};
        if (value != 0.0) {
            sum_exponent = std::max(sum_exponent, terms[axis].exponent);
        }
    }
    if (sum_exponent == std::numeric_limits<int>::min()) {
        return EvaluationError::NumericalFailure;
    }
    // Neumaier compensation recovers the remainder of each aligned addition.
    // The larger operand comes first in the subtraction (FastTwoSum).
    double sum = 0.0;
    double correction = 0.0;
    for (const auto [high, low, exponent] : terms) {
        for (double part : {high, low}) {
            const double term = std::scalbn(part, exponent - sum_exponent);
            const double next = sum + term;
            correction += std::abs(sum) >= std::abs(term)
                ? (sum - next) + term : (term - next) + sum;
            sum = next;
        }
    }
    const double score = restoreScore(sum, correction, sum_exponent + 2 * common_exponent);
    if (!std::isfinite(score) || score == 0.0) {
        return EvaluationError::NumericalFailure;
    }
    return ConsistencyScore{score, Dimension};
}

} // namespace

std::variant<RmseResult, RmseFailure> computeRmse(std::span<const EstimateTruthPair> samples) {
    if (samples.empty()) {
        return RmseFailure{EvaluationError::EmptySamples, std::nullopt};
    }

    const auto epoch = samples.front().estimate.timestamp().epoch;
    VectorSquares position;
    VectorSquares velocity;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& [estimate, truth] = samples[index];
        const auto estimate_time = estimate.timestamp();
        const auto truth_time = truth.timestamp();
        if (estimate_time.epoch != epoch || truth_time.epoch != epoch) {
            return RmseFailure{EvaluationError::EpochMismatch, index};
        }
        if (estimate_time.elapsed != truth_time.elapsed) {
            return RmseFailure{EvaluationError::TimestampMismatch, index};
        }
        position.add(estimate.positionWorldM(), truth.positionWorldM());
        velocity.add(estimate.velocityWorldMps(), truth.linearVelocityWorldMps());
    }

    RmseResult result;
    result.sample_count = samples.size();
    if (!position.finish(result.sample_count, result.position_axis_m, result.position_3d_m) ||
        !velocity.finish(result.sample_count, result.velocity_axis_mps, result.velocity_3d_mps)) {
        return RmseFailure{EvaluationError::NumericalFailure, std::nullopt};
    }
    return result;
}

ScoreResult computeNees(const EstimatorState& estimate, const VehicleState& truth,
                        NeesComponents components) {
    const auto estimate_time = estimate.timestamp();
    const auto truth_time = truth.timestamp();
    if (estimate_time.epoch != truth_time.epoch) {
        return EvaluationError::EpochMismatch;
    }
    if (estimate_time.elapsed != truth_time.elapsed) {
        return EvaluationError::TimestampMismatch;
    }

    const auto covariance = estimate.covariance();
    const auto position = estimate.positionWorldM();
    const auto velocity = estimate.velocityWorldMps();
    const auto truth_position = truth.positionWorldM();
    const auto truth_velocity = truth.linearVelocityWorldMps();
    std::array<BinaryValue, 3> position_error;
    std::array<BinaryValue, 3> velocity_error;
    for (int axis = 0; axis < 3; ++axis) {
        position_error[axis] = difference(position(axis), truth_position(axis));
        velocity_error[axis] = difference(velocity(axis), truth_velocity(axis));
    }
    switch (components) {
    case NeesComponents::FullState:
        return whitenedScore<6>(covariance, {position_error[0], position_error[1], position_error[2],
                                             velocity_error[0], velocity_error[1], velocity_error[2]});
    case NeesComponents::Position:
        return whitenedScore<3>(covariance.topLeftCorner<3, 3>(), position_error);
    case NeesComponents::Velocity:
        return whitenedScore<3>(covariance.bottomRightCorner<3, 3>(), velocity_error);
    }
    return EvaluationError::NumericalFailure;
}

ScoreResult computeGpsNis(const EstimatorState& prior, const GpsMeasurement& measurement) {
    const auto prior_time = prior.timestamp();
    const auto acquisition = measurement.acquisition().timestamp();
    if (prior_time.epoch != acquisition.epoch) {
        return EvaluationError::EpochMismatch;
    }
    if (prior_time.elapsed != acquisition.elapsed) {
        return EvaluationError::TimestampMismatch;
    }

    const Vector3 innovation = measurement.positionWorldM() - prior.positionWorldM();
    const auto covariance = prior.covariance();
    const Eigen::Matrix3d innovation_covariance =
        covariance.topLeftCorner<3, 3>() + measurement.positionCovarianceWorldM2();
    if (!innovation.allFinite() || !innovation_covariance.allFinite()) {
        return EvaluationError::NumericalFailure;
    }

    std::array<BinaryValue, 3> residual;
    for (int axis = 0; axis < 3; ++axis) {
        int exponent;
        const double fraction = std::frexp(innovation(axis), &exponent);
        residual[axis] = {fraction, exponent};
    }
    return whitenedScore<3>(innovation_covariance, residual);
}

} // namespace aerotwin
