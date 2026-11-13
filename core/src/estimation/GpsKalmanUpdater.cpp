#include <aerotwin/estimation/GpsKalmanUpdater.hpp>

#include "../math/CovarianceValidation.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace aerotwin {
namespace {

// Keep double precision while carrying the binary exponent separately.
// Apple Silicon long double has the same exponent range as double.
struct Scaled {
    double fraction{};
    int exponent{};

    static Scaled from(double value, int shift = 0) {
        int exponent = 0;
        const double fraction = std::frexp(value, &exponent);
        return {fraction, exponent + shift};
    }

    double value() const { return std::scalbn(fraction, exponent); }
};

Scaled operator+(Scaled left, Scaled right) {
    if (left.fraction == 0.0) {
        return right;
    }
    if (right.fraction == 0.0) {
        return left;
    }
    if (left.exponent < right.exponent) {
        std::swap(left, right);
    }
    return Scaled::from(left.fraction +
                            std::scalbn(right.fraction, right.exponent - left.exponent),
                        left.exponent);
}

// Return a rounded sum and its rounding remainder. Keep the remainder
// separately when the exponents are too far apart for exact alignment.
std::pair<Scaled, Scaled> splitSum(Scaled left, Scaled right) {
    if (left.fraction == 0.0 || right.fraction == 0.0) {
        return {left + right, {}};
    }
    if (left.exponent < right.exponent) {
        std::swap(left, right);
    }
    constexpr int precision = std::numeric_limits<double>::digits;
    if (left.exponent - right.exponent > precision + 1) {
        return {left, right};
    }
    const double aligned = std::scalbn(right.fraction, right.exponent - left.exponent);
    const double sum = left.fraction + aligned;
    return {Scaled::from(sum, left.exponent),
            Scaled::from(aligned - (sum - left.fraction), left.exponent)};
}

Scaled operator*(Scaled left, Scaled right) {
    return Scaled::from(left.fraction * right.fraction, left.exponent + right.exponent);
}

Scaled operator/(Scaled numerator, Scaled denominator) {
    return Scaled::from(numerator.fraction / denominator.fraction,
                        numerator.exponent - denominator.exponent);
}

// Sum the prior velocity and the two parts of each axis contribution.
// Keeping each addition's remainder allows later terms to cancel without
// erasing earlier small contributions.
double correctedVelocity(double prior, const std::array<Scaled, 3>& gain,
                         const Vector3& position, const Vector3& observation) {
    std::array<Scaled, 7> partials{Scaled::from(prior)};
    std::size_t count = 1;
    for (int axis = 0; axis < 3; ++axis) {
        const auto [difference, remainder] =
            splitSum(Scaled::from(observation(axis)), Scaled::from(-position(axis)));
        for (Scaled term : {gain[axis] * difference, gain[axis] * remainder}) {
            for (std::size_t i = 0; i < count; ++i) {
                const auto [sum, error] = splitSum(term, partials[i]);
                partials[i] = error;
                term = sum;
            }
            partials[count++] = term;
        }
    }
    Scaled result;
    for (const auto part : partials) {
        result = result + part;
    }
    return result.value();
}

// One entry of A B Aᵀ, with the left product kept in the extended exponent range.
template <std::size_t N, typename Matrix>
Scaled congruenceEntry(const std::array<Scaled, N>& left, const Matrix& matrix,
                       const std::array<Scaled, N>& right) {
    Scaled entry;
    for (std::size_t column = 0; column < N; ++column) {
        Scaled product;
        for (std::size_t row = 0; row < N; ++row) {
            product = product + left[row] * Scaled::from(matrix(row, column));
        }
        entry = entry + product * right[column];
    }
    return entry;
}

} // namespace

GpsUpdateResult GpsKalmanUpdater::update(const EstimatorState& prior,
                                        const GpsMeasurement& measurement) {
    const auto timestamp = prior.timestamp();
    const auto acquisition = measurement.acquisition().timestamp();
    if (acquisition.epoch != timestamp.epoch) {
        return GpsUpdateError::EpochMismatch;
    }
    if (acquisition.elapsed != timestamp.elapsed) {
        return GpsUpdateError::TimestampMismatch;
    }

    // H = [I, 0], so Hx and HPHᵀ select the position vector and block.
    const Vector3 position = prior.positionWorldM();
    const Vector3 observation = measurement.positionWorldM();
    const Vector3 innovation = observation - position;
    const EstimatorCovariance6 covariance = prior.covariance();
    const Eigen::Matrix3d noise = measurement.positionCovarianceWorldM2();
    const Eigen::Matrix3d innovation_covariance = covariance.topLeftCorner<3, 3>() + noise;
    if (!innovation.allFinite() || !innovation_covariance.allFinite()) {
        return GpsUpdateError::NumericalFailure;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> spectrum;
    const auto symmetric =
        detail::validateCovariance(innovation_covariance, spectrum, Eigen::EigenvaluesOnly);
    if (!symmetric) {
        return GpsUpdateError::InvalidInnovationCovariance;
    }
    const Vector3 variances = symmetric->diagonal();
    // The shared validator computes the spectrum in correlation space. A
    // zero variance uses a unit placeholder there, so check it separately.
    constexpr double relative_floor = 3 * std::numeric_limits<double>::epsilon();
    if ((variances.array() == 0.0).any() ||
        spectrum.eigenvalues().minCoeff() <= relative_floor * spectrum.eigenvalues().maxCoeff()) {
        return GpsUpdateError::UnsolvableInnovationCovariance;
    }

    // S = D C D. Solve C (D Kᵀ) = D⁻¹ H P, then divide by D again.
    Eigen::Matrix3d correlation = Eigen::Matrix3d::Identity();
    for (int row = 1; row < 3; ++row) {
        for (int column = 0; column < row; ++column) {
            correlation(row, column) = correlation(column, row) =
                detail::normalizedEntry((*symmetric)(row, column), variances(row), variances(column));
        }
    }
    const Eigen::LLT<Eigen::Matrix3d> factor(correlation);
    if (factor.info() != Eigen::Success) {
        return GpsUpdateError::UnsolvableInnovationCovariance;
    }
    const Vector3 scales = variances.cwiseSqrt();
    // Normalize each RHS column before LLT. Use one exponent per column
    // so the solve still sees the original relative weights.
    // The last three columns solve for the transpose of R S⁻¹.
    Eigen::Matrix<double, 3, 9> numerators;
    numerators.leftCols<6>() = covariance.topRows<3>();
    numerators.rightCols<3>() = noise;
    std::array<int, 9> rhs_exponents{};
    Eigen::Matrix<double, 3, 9> rhs;
    for (int column = 0; column < rhs.cols(); ++column) {
        std::array<Scaled, 3> entries;
        int exponent = std::numeric_limits<int>::min();
        for (int row = 0; row < 3; ++row) {
            entries[row] = Scaled::from(numerators(row, column)) / Scaled::from(scales(row));
            if (entries[row].fraction != 0.0) {
                exponent = std::max(exponent, entries[row].exponent);
            }
        }
        rhs_exponents[column] = exponent == std::numeric_limits<int>::min() ? 0 : exponent;
        for (int row = 0; row < 3; ++row) {
            auto entry = entries[row];
            entry.exponent -= rhs_exponents[column];
            rhs(row, column) = entry.value();
            if (entry.fraction != 0.0 &&
                std::abs(rhs(row, column)) < std::numeric_limits<double>::min()) {
                return GpsUpdateError::NumericalFailure;
            }
        }
    }
    const Eigen::Matrix<double, 3, 9> solved = factor.solve(rhs);
    if (!solved.allFinite()) {
        return GpsUpdateError::NumericalFailure;
    }

    const auto weight = [&](int axis, int column) {
        return Scaled::from(solved(axis, column), rhs_exponents[column])
               / Scaled::from(scales(axis));
    };
    const bool exact_position = noise.isZero(0.0);
    std::array<std::array<Scaled, 3>, 6> gain{};
    std::array<std::array<Scaled, 6>, 6> residual{};
    EstimatorVector6 state = prior.vector();
    for (int row = 0; row < 6; ++row) {
        Scaled prior_term;
        Scaled observation_term;
        for (int axis = 0; axis < 3; ++axis) {
            gain[row][axis] = exact_position && row < 3
                ? Scaled::from(row == axis ? 1.0 : 0.0)
                : weight(axis, row);
            if (row < 3) {
                // I - Ppp S⁻¹ = R S⁻¹. Solve for the small weight directly.
                residual[row][axis] = weight(axis, row + 6);
            } else {
                residual[row][axis] = gain[row][axis];
                residual[row][axis].fraction = -residual[row][axis].fraction;
            }
            if (row < 3) {
                prior_term = prior_term + residual[row][axis] * Scaled::from(position(axis));
                observation_term = observation_term + gain[row][axis] * Scaled::from(observation(axis));
            }
        }
        if (row >= 3) {
            residual[row][row] = Scaled::from(1.0);
        }
        if (!innovation.isZero(0.0)) {
            state(row) = row < 3 ? (prior_term + observation_term).value()
                                 : correctedVelocity(state(row), gain[row], position, observation);
        }
    }

    // Round each entry after summing both Joseph terms, then mirror it.
    EstimatorCovariance6 posterior;
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column <= row; ++column) {
            const Scaled entry = congruenceEntry(residual[row], covariance, residual[column])
                                 + congruenceEntry(gain[row], noise, gain[column]);
            posterior(row, column) = posterior(column, row) = entry.value();
        }
    }
    if (!state.allFinite() || !posterior.allFinite()) {
        return GpsUpdateError::NumericalFailure;
    }
    const auto result = EstimatorState::create(state, posterior, timestamp);
    if (const auto* updated = std::get_if<EstimatorState>(&result)) {
        return *updated;
    }
    return GpsUpdateError::InvalidPosteriorCovariance;
}

} // namespace aerotwin
