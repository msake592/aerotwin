#include <aerotwin/math/Transformations.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

namespace {
using aerotwin::Quaternion;
using aerotwin::Vector3;
using aerotwin::bodyToWorldPoint;
using aerotwin::bodyToWorldVector;
using aerotwin::worldToBodyPoint;
using aerotwin::worldToBodyVector;

const Quaternion identity = Quaternion::Identity();
const Vector3 zero = Vector3::Zero();
const Quaternion yaw90(std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5));

void expectVector(const std::optional<Vector3>& actual, const Vector3& expected,
                  double tolerance = 1e-12) {
    ASSERT_TRUE(actual.has_value());
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR((*actual)[axis], expected[axis], tolerance);
    }
}

TEST(Transformations, IdentityPreservesPointsAndVectors) {
    const Vector3 value(2.0, -3.0, 4.0);
    expectVector(bodyToWorldVector(identity, value), value);
    expectVector(worldToBodyVector(identity, value), value);
    expectVector(bodyToWorldPoint(identity, zero, value), value);
    expectVector(worldToBodyPoint(identity, zero, value), value);
}

TEST(Transformations, PositiveYawMapsForwardNorthAndLeftWest) {
    // At identity FLU forward/east, left/north, up/up are aligned.
    const std::array<Vector3, 3> body{Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ()};
    const std::array<Vector3, 3> world{Vector3::UnitY(), -Vector3::UnitX(), Vector3::UnitZ()};
    for (int axis = 0; axis < 3; ++axis) {
        expectVector(bodyToWorldVector(identity, body[axis]), body[axis]);
        expectVector(bodyToWorldVector(yaw90, body[axis]), world[axis]);
        expectVector(worldToBodyVector(yaw90, world[axis]), body[axis]);
    }
}

TEST(Transformations, RollAndPitchRespectRightHandedFluAxes) {
    const double s = std::sqrt(0.5);
    expectVector(bodyToWorldVector(Quaternion(s, s, 0, 0), Vector3::UnitY()), Vector3::UnitZ());
    expectVector(bodyToWorldVector(Quaternion(s, 0, s, 0), Vector3::UnitX()), -Vector3::UnitZ());
}

TEST(Transformations, PointsTranslateAfterRotationAndInverseSubtractsBeforeRotation) {
    const Vector3 position(10, -20, 30);
    const Vector3 body(2, 3, 4);
    const Vector3 world(7, -18, 34);
    expectVector(bodyToWorldPoint(yaw90, position, body), world);
    expectVector(worldToBodyPoint(yaw90, position, world), body);
    expectVector(bodyToWorldVector(yaw90, body), Vector3(-3, 2, 4));
    expectVector(worldToBodyVector(yaw90, Vector3(-3, 2, 4)), body);
    expectVector(bodyToWorldPoint(yaw90, position, zero), position);
    expectVector(worldToBodyPoint(yaw90, position, position), zero);
}

TEST(Transformations, TranslationOnly) {
    const Vector3 position(10, 20, -30);
    const Vector3 body(-2, 3, 4);
    expectVector(bodyToWorldPoint(identity, position, body), Vector3(8, 23, -26));
    expectVector(worldToBodyPoint(identity, position, Vector3(8, 23, -26)), body);
}

TEST(Transformations, ArbitraryAttitudesRoundTripInBothDirections) {
    const std::array<Quaternion, 3> attitudes{
        identity, yaw90, Quaternion(Eigen::AngleAxisd(1.234, Vector3(1, -2, 3).normalized()))};
    const Vector3 position(10, -20, 30);
    const Vector3 value(0.25, -3.5, 12);
    for (const auto& q : attitudes) {
        const auto world_vector = bodyToWorldVector(q, value);
        const auto body_vector = worldToBodyVector(q, value);
        const auto world_point = bodyToWorldPoint(q, position, value);
        const auto body_point = worldToBodyPoint(q, position, value);
        ASSERT_TRUE(world_vector && body_vector && world_point && body_point);
        expectVector(worldToBodyVector(q, *world_vector), value);
        expectVector(bodyToWorldVector(q, *body_vector), value);
        expectVector(worldToBodyPoint(q, position, *world_point), value);
        expectVector(bodyToWorldPoint(q, position, *body_point), value);
        Quaternion negative;
        negative.coeffs() = -q.coeffs();
        expectVector(bodyToWorldPoint(negative, position, value), *world_point);
    }
}

TEST(Transformations, RejectsInvalidQuaternionsInEveryOperation) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::array<Quaternion, 6> invalid{
        Quaternion(0, 0, 0, 0), Quaternion(2, 0, 0, 0), Quaternion(1 + 2e-12, 0, 0, 0),
        Quaternion(nan, 0, 0, 0), Quaternion(1, inf, 0, 0), Quaternion(1, 0, -inf, 0)};
    for (const auto& q : invalid) {
        EXPECT_FALSE(bodyToWorldVector(q, zero));
        EXPECT_FALSE(worldToBodyVector(q, zero));
        EXPECT_FALSE(bodyToWorldPoint(q, zero, zero));
        EXPECT_FALSE(worldToBodyPoint(q, zero, zero));
    }
}

TEST(Transformations, ReusesAcceptedQuaternionDriftPolicyWithoutMutatingInputs) {
    Quaternion drift = yaw90;
    drift.coeffs() *= 1 + 5e-13;
    const Quaternion original = drift;
    expectVector(bodyToWorldVector(drift, Vector3::UnitX()), Vector3::UnitY());
    expectVector(worldToBodyVector(drift, Vector3::UnitY()), Vector3::UnitX());
    expectVector(bodyToWorldPoint(drift, Vector3(1, 2, 3), Vector3::UnitX()), Vector3(1, 3, 3));
    expectVector(worldToBodyPoint(drift, Vector3(1, 2, 3), Vector3(1, 3, 3)), Vector3::UnitX());
    EXPECT_EQ(drift.coeffs(), original.coeffs());
}

TEST(Transformations, RejectsNonfiniteComponentsInEveryVectorAndPositionArgument) {
    const double inf = std::numeric_limits<double>::infinity();
    for (double bad : {std::numeric_limits<double>::quiet_NaN(), inf, -inf}) {
        for (int axis = 0; axis < 3; ++axis) {
            Vector3 value = zero;
            value[axis] = bad;
            EXPECT_FALSE(bodyToWorldVector(identity, value));
            EXPECT_FALSE(worldToBodyVector(identity, value));
            EXPECT_FALSE(bodyToWorldPoint(identity, zero, value));
            EXPECT_FALSE(bodyToWorldPoint(identity, value, zero));
            EXPECT_FALSE(worldToBodyPoint(identity, zero, value));
            EXPECT_FALSE(worldToBodyPoint(identity, value, zero));
        }
    }
}

TEST(Transformations, RejectsTranslationAndSubtractionOverflow) {
    const double max = std::numeric_limits<double>::max();
    EXPECT_FALSE(bodyToWorldPoint(identity, Vector3(max, 0, 0), Vector3(max, 0, 0)));
    EXPECT_FALSE(worldToBodyPoint(identity, Vector3(-max, 0, 0), Vector3(max, 0, 0)));
    // This overflowing subtraction precedes a rotation which could spread its magnitude.
    const Quaternion q(Eigen::AngleAxisd(std::acos(-1.0) / 4, Vector3::UnitZ()));
    EXPECT_FALSE(worldToBodyPoint(q, Vector3(-0.6 * max, 0, 0), Vector3(0.6 * max, 0, 0)));
}

TEST(Transformations, RejectsRotationOverflowEvenWhenLaterTranslationCouldCancel) {
    const double max = std::numeric_limits<double>::max();
    const Quaternion q(Eigen::AngleAxisd(std::acos(-1.0) / 4, Vector3::UnitZ()));
    const Vector3 large(max, -max, 0);
    EXPECT_FALSE(bodyToWorldVector(q, large));
    EXPECT_FALSE(bodyToWorldPoint(q, Vector3(-max, 0, 0), large));
    EXPECT_FALSE(worldToBodyVector(q, Vector3(max, max, 0)));
    EXPECT_FALSE(worldToBodyPoint(q, zero, Vector3(max, max, 0)));
}

TEST(Transformations, RecoversInternalRotationOverflowWithZeroTranslation) {
    // M1.3 regression scenario: a half turn around (1,1,1) fixes this vector,
    // but unscaled rotation partial sums overflow before cancellation.
    const double component = 1.0 / std::sqrt(3.0);
    const Quaternion rotation(0.0, component, component, component);
    const double large = 0.9 * std::numeric_limits<double>::max();
    const Vector3 value(large, large, large);
    const std::array<std::optional<Vector3>, 4> results{
        bodyToWorldVector(rotation, value), worldToBodyVector(rotation, value),
        bodyToWorldPoint(rotation, zero, value), worldToBodyPoint(rotation, zero, value)};
    for (const auto& result : results) {
        ASSERT_TRUE(result.has_value());
        ASSERT_TRUE(aerotwin::isFinite(*result));
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_NEAR((*result)[axis] / large, 1.0, 1e-12);
        }
    }
}

TEST(Transformations, PreservesMixedMagnitudeAndSubnormalComponentsWhenRepresentable) {
    const double tiny = std::numeric_limits<double>::denorm_min();
    const Vector3 value(1e300, 1e-100, tiny);
    const auto forward = bodyToWorldPoint(identity, zero, value);
    const auto reverse = worldToBodyPoint(identity, zero, value);
    ASSERT_TRUE(forward && reverse);
    EXPECT_EQ(*forward, value);
    EXPECT_EQ(*reverse, value);
    const Vector3 translation(tiny, -tiny, 0);
    expectVector(bodyToWorldPoint(identity, translation, translation), Vector3(2 * tiny, -2 * tiny, 0), 0);
    expectVector(worldToBodyPoint(identity, translation, zero), -translation, 0);
}

TEST(Transformations, FiniteCancellationAndRoundedAwayTranslationFollowDoubleArithmetic) {
    const double max = std::numeric_limits<double>::max();
    expectVector(bodyToWorldPoint(identity, Vector3(-max, 0, 0), Vector3(max, 0, 0)), zero, 0);
    expectVector(worldToBodyPoint(identity, Vector3(max, 0, 0), Vector3(max, 0, 0)), zero, 0);
    // A unit offset cannot be retained at 1e300. Round trips cannot recover lost bits.
    const auto world = bodyToWorldPoint(identity, Vector3(1e300, 0, 0), Vector3::UnitX());
    ASSERT_TRUE(world);
    EXPECT_EQ(world->x(), 1e300);
    expectVector(worldToBodyPoint(identity, Vector3(1e300, 0, 0), *world), zero, 0);
}
} // namespace
