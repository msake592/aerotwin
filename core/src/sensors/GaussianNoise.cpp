#include <aerotwin/sensors/GaussianNoise.hpp>

#include "Covariance.hpp"

#include <cmath>
#include <numbers>

namespace aerotwin {
namespace {

// Map the high 52 bits to exact binary midpoints strictly between zero and one.
double openUniform(std::mt19937_64& engine) noexcept {
    return (static_cast<double>(engine() >> 12) + 0.5) * 0x1p-52;
}

} // namespace

GaussianNoise::GaussianNoise(std::uint64_t seed) noexcept : engine_(seed) {}

void GaussianNoise::reseed(std::uint64_t seed) noexcept {
    engine_.seed(seed);
}

double GaussianNoise::standardNormal() noexcept {
    const double radius_uniform = openUniform(engine_);
    const double angle_uniform = openUniform(engine_);
    return std::sqrt(-2.0 * std::log(radius_uniform)) *
           std::cos(2.0 * std::numbers::pi * angle_uniform);
}

std::optional<double> GaussianNoise::sample(double standard_deviation) noexcept {
    if (!std::isfinite(standard_deviation) || standard_deviation < 0.0) {
        return std::nullopt;
    }
    if (standard_deviation == 0.0) {
        return 0.0;
    }
    const double value = standard_deviation * standardNormal();
    if (!std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

std::optional<GaussianNoise3> GaussianNoise3::create(const Eigen::Matrix3d& covariance) {
    Eigen::Matrix3d factor;
    if (!detail::validatedCovariance(covariance, &factor)) {
        return std::nullopt;
    }
    return GaussianNoise3(factor);
}

std::optional<Vector3> GaussianNoise3::sample(GaussianNoise& stream) const noexcept {
    const double x = stream.standardNormal();
    const double y = stream.standardNormal();
    const double z = stream.standardNormal();
    const Vector3 value = factor_ * Vector3(x, y, z);
    if (!isFinite(value)) {
        return std::nullopt;
    }
    return value;
}

} // namespace aerotwin
