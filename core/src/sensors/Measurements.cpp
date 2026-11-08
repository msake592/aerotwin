#include <aerotwin/sensors/Measurements.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace aerotwin {
namespace {

// Keep the geometric mean in mantissa/exponent form so a subnormal
// denominator is never rounded before division.
double normalizedEntry(double value, double variance_a, double variance_b) {
    int exponent_a, exponent_b, exponent_value;
    const double a = std::frexp(variance_a, &exponent_a);
    const double b = std::frexp(variance_b, &exponent_b);
    const double fraction = std::frexp(value, &exponent_value);
    const int sum = exponent_a + exponent_b;
    const int half = sum / 2;
    const double denominator = std::sqrt(std::scalbn(a * b, sum - 2 * half));
    return std::scalbn(fraction / denominator, exponent_value - half);
}

std::optional<Eigen::Matrix3d> validatedCovariance(const Eigen::Matrix3d& input) {
    if (!input.allFinite() || (input.diagonal().array() < 0.0).any()) {
        return std::nullopt;
    }

    constexpr double tolerance = 1e-12;
    Eigen::Matrix3d symmetric = input;
    Eigen::Matrix3d correlation = Eigen::Matrix3d::Identity();
    for (int row = 1; row < 3; ++row) {
        for (int column = 0; column < row; ++column) {
            const double lower = input(row, column);
            const double upper = input(column, row);
            const double variance_a = input(row, row);
            const double variance_b = input(column, column);
            if (variance_a == 0.0 || variance_b == 0.0) {
                if (lower != 0.0 || upper != 0.0) {
                    return std::nullopt;
                }
                continue;
            }

            const double a = normalizedEntry(lower, variance_a, variance_b);
            const double b = normalizedEntry(upper, variance_a, variance_b);
            if (!std::isfinite(a) || !std::isfinite(b)) {
                return std::nullopt;
            }
            const double pair_scale = std::max({1.0, std::abs(a), std::abs(b)});
            if (std::abs(a / pair_scale - b / pair_scale) > tolerance) {
                return std::nullopt;
            }

            const double mean = std::midpoint(lower, upper);
            const double normalized = normalizedEntry(mean, variance_a, variance_b);
            if (!std::isfinite(normalized) || std::abs(normalized) > 1.0 + tolerance) {
                return std::nullopt;
            }
            symmetric(row, column) = symmetric(column, row) = mean;
            correlation(row, column) = correlation(column, row) = normalized;
        }
    }
    // A zero-variance axis is isolated. A unit placeholder on that axis
    // leaves the PSD decision on the positive-variance block unchanged.
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(correlation, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
        solver.eigenvalues().minCoeff() < -tolerance) {
        return std::nullopt;
    }
    return symmetric;
}

} // namespace

std::optional<MeasurementAcquisition> MeasurementAcquisition::create(
    std::string_view sensor_id, SimulationTimestamp timestamp) noexcept {
    constexpr std::string_view allowed =
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-";
    if (sensor_id.empty() || sensor_id.size() > 64 ||
        sensor_id.find_first_not_of(allowed) != std::string_view::npos ||
        timestamp.elapsed < SimulationNanoseconds::zero()) {
        return std::nullopt;
    }
    return MeasurementAcquisition(sensor_id, timestamp);
}

MeasurementAcquisition::MeasurementAcquisition(std::string_view sensor_id,
                                               SimulationTimestamp timestamp) noexcept
    : size_(sensor_id.size()), timestamp_(timestamp) {
    std::copy(sensor_id.begin(), sensor_id.end(), sensor_id_.begin());
}

std::optional<GpsMeasurement> GpsMeasurement::create(
    MeasurementAcquisition acquisition, const Vector3& position_world_m,
    const Eigen::Matrix3d& position_covariance_world_m2) {
    if (!isFinite(position_world_m)) {
        return std::nullopt;
    }
    const auto covariance = validatedCovariance(position_covariance_world_m2);
    if (!covariance) {
        return std::nullopt;
    }
    return GpsMeasurement(acquisition, position_world_m, *covariance);
}

GpsMeasurement::GpsMeasurement(MeasurementAcquisition acquisition, const Vector3& position_world_m,
                               const Eigen::Matrix3d& covariance) noexcept
    : acquisition_(acquisition), position_world_m_(position_world_m),
      position_covariance_world_m2_(covariance) {}

std::optional<ImuMeasurement> ImuMeasurement::create(
    MeasurementAcquisition acquisition, const Vector3& specific_force_body_mps2,
    const Vector3& angular_velocity_body_radps,
    const Eigen::Matrix3d& specific_force_covariance_body_m2ps4,
    const Eigen::Matrix3d& angular_velocity_covariance_body_rad2ps2) {
    if (!isFinite(specific_force_body_mps2) || !isFinite(angular_velocity_body_radps)) {
        return std::nullopt;
    }
    const auto force_covariance = validatedCovariance(specific_force_covariance_body_m2ps4);
    const auto rate_covariance = validatedCovariance(angular_velocity_covariance_body_rad2ps2);
    if (!force_covariance || !rate_covariance) {
        return std::nullopt;
    }
    return ImuMeasurement(acquisition, specific_force_body_mps2, angular_velocity_body_radps,
                          *force_covariance, *rate_covariance);
}

ImuMeasurement::ImuMeasurement(MeasurementAcquisition acquisition,
                               const Vector3& specific_force_body_mps2,
                               const Vector3& angular_velocity_body_radps,
                               const Eigen::Matrix3d& specific_force_covariance,
                               const Eigen::Matrix3d& angular_velocity_covariance) noexcept
    : acquisition_(acquisition), specific_force_body_mps2_(specific_force_body_mps2),
      angular_velocity_body_radps_(angular_velocity_body_radps),
      specific_force_covariance_body_m2ps4_(specific_force_covariance),
      angular_velocity_covariance_body_rad2ps2_(angular_velocity_covariance) {}

} // namespace aerotwin
