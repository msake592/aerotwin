#include <aerotwin/sensors/GaussianNoise.hpp>

#include "../../src/sensors/Covariance.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <utility>

namespace aerotwin {
namespace {

TEST(GaussianNoiseTest, SeedReplayAndIndependentSnapshots) {
    GaussianNoise first(42);
    GaussianNoise second(42);
    GaussianNoise other(43);
    const double initial = first.standardNormal();
    EXPECT_EQ(initial, second.standardNormal());
    EXPECT_NE(initial, other.standardNormal());

    auto copy = first;
    auto from_rvalue = std::move(first);
    for (int i = 0; i < 16; ++i) {
        const double value = second.standardNormal();
        EXPECT_EQ(first.standardNormal(), value);
        EXPECT_EQ(copy.standardNormal(), value);
        EXPECT_EQ(from_rvalue.standardNormal(), value);
    }

    first.reseed(42);
    EXPECT_EQ(first.standardNormal(), initial);
    first.reseed(43);
    EXPECT_EQ(first.standardNormal(), GaussianNoise(43).standardNormal());
}

TEST(GaussianNoiseTest, MatchesGoldenBoxMullerSequence) {
    GaussianNoise stream(5489);
    constexpr std::array expected{-0.0020899072880718882, 0.78052242289445661,
                                  -2.3234487295764374, 1.6450546114326348};
    for (double value : expected) {
        EXPECT_NEAR(stream.standardNormal(), value, 2e-14);
    }
}

TEST(GaussianNoiseTest, ScalarValidationAndConsumption) {
    GaussianNoise stream(5489);
    GaussianNoise reference(5489);
    EXPECT_EQ(stream.sample(0.0), 0.0);
    EXPECT_FALSE(stream.sample(-1.0));
    EXPECT_FALSE(stream.sample(std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(stream.sample(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_EQ(stream.sample(2.0), 2.0 * reference.standardNormal());

    EXPECT_EQ(stream.standardNormal(), reference.standardNormal());
    // The third reference draw is less than -2 and must overflow at this scale.
    EXPECT_LT(reference.standardNormal(), -2.0);
    EXPECT_FALSE(stream.sample(std::numeric_limits<double>::max()));
    EXPECT_EQ(stream.standardNormal(), reference.standardNormal());
}

TEST(GaussianNoiseTest, ReplaysVectorSamplesAndZeroCovarianceConsumesThreeDraws) {
    const auto model = GaussianNoise3::create(Eigen::Matrix3d::Identity());
    const auto zero = GaussianNoise3::create(Eigen::Matrix3d::Zero());
    ASSERT_TRUE(model);
    ASSERT_TRUE(zero);
    GaussianNoise first(23);
    GaussianNoise second(23);
    const auto a = model->sample(first);
    const auto b = model->sample(second);
    ASSERT_TRUE(a);
    ASSERT_TRUE(b);
    EXPECT_EQ(*a, *b);

    const auto zero_sample = zero->sample(first);
    ASSERT_TRUE(zero_sample);
    EXPECT_EQ(*zero_sample, Vector3::Zero());
    for (int i = 0; i < 3; ++i) {
        static_cast<void>(second.standardNormal());
    }
    EXPECT_EQ(first.standardNormal(), second.standardNormal());
}

TEST(GaussianNoiseTest, SupportsSingularAndExtremeMixedScaleCovariances) {
    Eigen::Matrix3d covariance;
    covariance << 4.0, -6.0, 0.0,
                 -6.0, 9.0, 0.0,
                  0.0, 0.0, 0.0;
    const auto singular = GaussianNoise3::create(covariance);
    ASSERT_TRUE(singular);
    GaussianNoise stream(76);
    for (int i = 0; i < 16; ++i) {
        const auto value = singular->sample(stream);
        ASSERT_TRUE(value);
        EXPECT_NEAR(value->x() / 2.0, -value->y() / 3.0, 1e-7);
        EXPECT_EQ(value->z(), 0.0);
    }

    covariance << 1e300, 0.5, 0.0,
                  0.5, 1e-300, 0.0,
                  0.0, 0.0, 0.0;
    const auto mixed = GaussianNoise3::create(covariance);
    ASSERT_TRUE(mixed);
    const auto sample = mixed->sample(stream);
    ASSERT_TRUE(sample);
    EXPECT_TRUE(isFinite(*sample));
    EXPECT_NE(sample->x(), 0.0);
    EXPECT_NE(sample->y(), 0.0);
    EXPECT_EQ(sample->z(), 0.0);
}

TEST(GaussianNoiseTest, ReconstructsMixedScaleAndRankTwoCovariances) {
    Eigen::Matrix3d mixed_covariance;
    mixed_covariance << 1e300, 0.5, 0.0,
                        0.5, 1e-300, 0.0,
                        0.0, 0.0, 4.0;
    Eigen::Matrix3d mixed_correlation = Eigen::Matrix3d::Identity();
    mixed_correlation(0, 1) = mixed_correlation(1, 0) = 0.5;

    // This correlation matrix has eigenvalues 0, 1, and 2.
    Eigen::Matrix3d rank_two_correlation;
    rank_two_correlation << 1.0, 0.6, 0.8,
                            0.6, 1.0, 0.0,
                            0.8, 0.0, 1.0;
    Eigen::Matrix3d rank_two_covariance;
    rank_two_covariance << 4.0, 6.0, 16.0,
                           6.0, 25.0, 0.0,
                           16.0, 0.0, 100.0;

    const auto check_reconstruction = [](const Eigen::Matrix3d& covariance,
                                         const Vector3& standard_deviations,
                                         const Eigen::Matrix3d& expected_correlation) {
        Eigen::Matrix3d factor;
        ASSERT_TRUE(detail::validatedCovariance(covariance, &factor));

        // Undo the expected physical row scales before squaring the factor.
        // The tolerance allows rounding in the 3x3 eigensolve and reconstruction.
        for (int row = 0; row < 3; ++row) {
            factor.row(row) /= standard_deviations(row);
        }
        const Eigen::Matrix3d reconstructed = factor * factor.transpose();
        EXPECT_LE((reconstructed - expected_correlation).cwiseAbs().maxCoeff(), 1e-14);
    };

    check_reconstruction(mixed_covariance, Vector3(1e150, 1e-150, 2.0), mixed_correlation);
    check_reconstruction(rank_two_covariance, Vector3(2.0, 5.0, 10.0), rank_two_correlation);
}

TEST(GaussianNoiseTest, RejectsInvalidCovariances) {
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
    covariance(0, 0) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(GaussianNoise3::create(covariance));
    covariance(0, 0) = -1.0;
    EXPECT_FALSE(GaussianNoise3::create(covariance));

    covariance << 1e16, 0.0, 0.0,
                  0.0, 1.0, 2.0,
                  0.0, 2.0, 1.0;
    EXPECT_FALSE(GaussianNoise3::create(covariance));
    covariance(1, 2) = 1.5;
    covariance(2, 1) = 0.0;
    EXPECT_FALSE(GaussianNoise3::create(covariance));
    covariance(1, 1) = 0.0;
    EXPECT_FALSE(GaussianNoise3::create(covariance));
}

TEST(GaussianNoiseTest, StatisticalSanityForDiagonalAndCorrelatedNoise) {
    const Eigen::Matrix3d diagonal = Vector3(1.0, 4.0, 9.0).asDiagonal();
    Eigen::Matrix3d correlated = diagonal;
    correlated(0, 1) = correlated(1, 0) = -1.2;
    correlated(0, 2) = correlated(2, 0) = 1.5;

    for (const auto& covariance : {diagonal, correlated}) {
        const auto model = GaussianNoise3::create(covariance);
        ASSERT_TRUE(model);
        GaussianNoise stream(275);
        Vector3 mean = Vector3::Zero();
        Eigen::Matrix3d second_moment = Eigen::Matrix3d::Zero();
        constexpr int samples = 30'000;
        for (int i = 0; i < samples; ++i) {
            const auto value = model->sample(stream);
            ASSERT_TRUE(value);
            mean += *value;
            second_moment += *value * value->transpose();
        }
        mean /= samples;
        const Eigen::Matrix3d estimate = second_moment / samples - mean * mean.transpose();
        for (int row = 0; row < 3; ++row) {
            const double sigma = std::sqrt(covariance(row, row));
            EXPECT_NEAR(mean(row), 0.0, 0.04 * sigma);
            for (int column = 0; column < 3; ++column) {
                // More than seven standard errors even for the diagonal estimates.
                const double scale = sigma * std::sqrt(covariance(column, column));
                EXPECT_NEAR(estimate(row, column), covariance(row, column), 0.06 * scale);
            }
        }
    }
}

} // namespace
} // namespace aerotwin
