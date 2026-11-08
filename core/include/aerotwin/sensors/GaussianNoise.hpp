#pragma once

#include <aerotwin/math/Types.hpp>

#include <cstdint>
#include <optional>
#include <random>

namespace aerotwin {

// Owns one stream. Copies and moves snapshot its current position.
class GaussianNoise {
public:
    explicit GaussianNoise(std::uint64_t seed) noexcept;
    GaussianNoise(const GaussianNoise&) = default;
    GaussianNoise& operator=(const GaussianNoise&) = default;

    void reseed(std::uint64_t seed) noexcept;
    [[nodiscard]] double standardNormal() noexcept;

    // Invalid or zero standard deviations consume no draws. Overflow after a
    // valid positive standard deviation consumes the draw and returns nullopt.
    [[nodiscard]] std::optional<double> sample(double standard_deviation) noexcept;

private:
    std::mt19937_64 engine_;
};

// Validates and factors covariance once. Sampling advances the caller's stream.
class GaussianNoise3 {
public:
    [[nodiscard]] static std::optional<GaussianNoise3> create(const Eigen::Matrix3d& covariance);

    // Every call consumes three normal draws, including for zero covariance.
    [[nodiscard]] std::optional<Vector3> sample(GaussianNoise& stream) const noexcept;

private:
    explicit GaussianNoise3(const Eigen::Matrix3d& factor) noexcept : factor_(factor) {}

    Eigen::Matrix3d factor_;
};

} // namespace aerotwin
