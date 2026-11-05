#include <aerotwin/math/Rotations.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using aerotwin::Quaternion;
using aerotwin::Vector3;
constexpr double kTolerance = 1e-12;

Quaternion quarterTurnZ() {
    const double half_sqrt_two = std::sqrt(0.5);
    return Quaternion(half_sqrt_two, 0.0, 0.0, half_sqrt_two);
}

Quaternion quarterTurnX() {
    const double half_sqrt_two = std::sqrt(0.5);
    return Quaternion(half_sqrt_two, half_sqrt_two, 0.0, 0.0);
}

void expectVectorNear(const Vector3& actual, const Vector3& expected) {
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR(actual[axis], expected[axis], kTolerance);
    }
}

} // namespace

TEST(RotationsTest, IdentityPreservesVectorAndZero) {
    const Vector3 vector(1.25, -2.5, 3.75);
    const auto result = aerotwin::rotateVector(Quaternion::Identity(), vector);
    ASSERT_TRUE(result);
    expectVectorNear(*result, vector);
    const auto zero = aerotwin::rotateVector(quarterTurnZ(), Vector3::Zero());
    ASSERT_TRUE(zero);
    EXPECT_TRUE(zero->isZero(0.0));
}

TEST(RotationsTest, NormalizesExplicitWXYZComponentsWithoutMutatingInput) {
    const Quaternion input(2.0, -2.0, 2.0, -2.0);
    const auto result = aerotwin::normalizeQuaternion(input);
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ(result->w(), 0.5);
    EXPECT_DOUBLE_EQ(result->x(), -0.5);
    EXPECT_DOUBLE_EQ(result->y(), 0.5);
    EXPECT_DOUBLE_EQ(result->z(), -0.5);
    EXPECT_TRUE(aerotwin::isUnitQuaternion(*result));
    EXPECT_DOUBLE_EQ(input.w(), 2.0);
}

TEST(RotationsTest, NormalizesExtremeFiniteMagnitudes) {
    for (double magnitude : {std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::min(),
                             std::numeric_limits<double>::denorm_min()}) {
        const auto result = aerotwin::normalizeQuaternion(
            Quaternion(magnitude, magnitude, -magnitude, magnitude));
        ASSERT_TRUE(result);
        EXPECT_DOUBLE_EQ(result->w(), 0.5);
        EXPECT_DOUBLE_EQ(result->x(), 0.5);
        EXPECT_DOUBLE_EQ(result->y(), -0.5);
        EXPECT_DOUBLE_EQ(result->z(), 0.5);
    }
}

TEST(RotationsTest, PositiveZQuarterTurnMapsXToY) {
    const auto result = aerotwin::rotateVector(quarterTurnZ(), Vector3::UnitX());
    ASSERT_TRUE(result);
    expectVectorNear(*result, Vector3::UnitY());
    const auto fixed_axis = aerotwin::rotateVector(quarterTurnZ(), Vector3::UnitZ());
    ASSERT_TRUE(fixed_axis);
    expectVectorNear(*fixed_axis, Vector3::UnitZ());
}

TEST(RotationsTest, InverseConjugatesAndReversesRotation) {
    const auto inverse = aerotwin::inverseRotation(quarterTurnZ());
    ASSERT_TRUE(inverse);
    EXPECT_NEAR(inverse->w(), std::sqrt(0.5), kTolerance);
    EXPECT_NEAR(inverse->z(), -std::sqrt(0.5), kTolerance);
    const auto result = aerotwin::rotateVector(*inverse, Vector3::UnitY());
    ASSERT_TRUE(result);
    expectVectorNear(*result, Vector3::UnitX());
    const auto identity = aerotwin::composeRotations(*inverse, quarterTurnZ());
    ASSERT_TRUE(identity);
    EXPECT_NEAR(identity->w(), 1.0, kTolerance);
    EXPECT_NEAR(identity->vec().norm(), 0.0, kTolerance);
}

TEST(RotationsTest, CompositionAppliesRightOperandFirstAndIsNotCommutative) {
    const auto x_then_z = aerotwin::composeRotations(quarterTurnZ(), quarterTurnX());
    const auto z_then_x = aerotwin::composeRotations(quarterTurnX(), quarterTurnZ());
    ASSERT_TRUE(x_then_z);
    ASSERT_TRUE(z_then_x);
    EXPECT_NEAR(x_then_z->w(), 0.5, kTolerance);
    EXPECT_NEAR(x_then_z->x(), 0.5, kTolerance);
    EXPECT_NEAR(x_then_z->y(), 0.5, kTolerance);
    EXPECT_NEAR(x_then_z->z(), 0.5, kTolerance);
    const auto first = aerotwin::rotateVector(*x_then_z, Vector3::UnitY());
    const auto reversed = aerotwin::rotateVector(*z_then_x, Vector3::UnitY());
    ASSERT_TRUE(first);
    ASSERT_TRUE(reversed);
    expectVectorNear(*first, Vector3::UnitZ());
    expectVectorNear(*reversed, -Vector3::UnitX());
    const auto intermediate = aerotwin::rotateVector(quarterTurnX(), Vector3::UnitY());
    ASSERT_TRUE(intermediate);
    const auto sequential = aerotwin::rotateVector(quarterTurnZ(), *intermediate);
    ASSERT_TRUE(sequential);
    expectVectorNear(*first, *sequential);
}

TEST(RotationsTest, RoundTripPreservesVectorAndRotationPreservesNorm) {
    const auto rotation = aerotwin::normalizeQuaternion(Quaternion(1.0, -2.0, 3.0, -4.0));
    ASSERT_TRUE(rotation);
    const auto inverse = aerotwin::inverseRotation(*rotation);
    ASSERT_TRUE(inverse);
    for (const Vector3& vector : {Vector3(1.25, -2.5, 3.75), Vector3::UnitX().eval(),
                                 Vector3::UnitY().eval(), Vector3::UnitZ().eval()}) {
        const auto rotated = aerotwin::rotateVector(*rotation, vector);
        ASSERT_TRUE(rotated);
        EXPECT_NEAR(rotated->norm(), vector.norm(), kTolerance);
        const auto recovered = aerotwin::rotateVector(*inverse, *rotated);
        ASSERT_TRUE(recovered);
        expectVectorNear(*recovered, vector);
    }
}

TEST(RotationsTest, QuaternionSignsProduceSameRotation) {
    const Quaternion positive = quarterTurnZ();
    const Quaternion negative(-positive.w(), -positive.x(), -positive.y(), -positive.z());
    const auto first = aerotwin::rotateVector(positive, Vector3::UnitX());
    const auto second = aerotwin::rotateVector(negative, Vector3::UnitX());
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    expectVectorNear(*first, *second);
    const auto normalized = aerotwin::normalizeQuaternion(negative);
    ASSERT_TRUE(normalized);
    EXPECT_LT(normalized->w(), 0.0);
}

TEST(RotationsTest, RejectsZeroAndNonFiniteQuaternionsInEveryOperation) {
    const auto reject = [](const Quaternion& invalid) {
        EXPECT_FALSE(aerotwin::normalizeQuaternion(invalid));
        EXPECT_FALSE(aerotwin::inverseRotation(invalid));
        EXPECT_FALSE(aerotwin::rotateVector(invalid, Vector3::UnitX()));
        EXPECT_FALSE(aerotwin::composeRotations(invalid, Quaternion::Identity()));
        EXPECT_FALSE(aerotwin::composeRotations(Quaternion::Identity(), invalid));
    };
    reject(Quaternion(0.0, 0.0, 0.0, 0.0));
    for (int axis = 0; axis < 4; ++axis) {
        for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
            Quaternion rotation = Quaternion::Identity();
            rotation.coeffs()[axis] = invalid;
            reject(rotation);
        }
    }
}

TEST(RotationsTest, RejectsScaledRotationsUntilExplicitlyNormalized) {
    const Quaternion scaled(2.0, 0.0, 0.0, 0.0);
    EXPECT_FALSE(aerotwin::inverseRotation(scaled));
    EXPECT_FALSE(aerotwin::rotateVector(scaled, Vector3::UnitX()));
    EXPECT_FALSE(aerotwin::composeRotations(scaled, Quaternion::Identity()));
    EXPECT_FALSE(aerotwin::composeRotations(Quaternion::Identity(), scaled));
    const auto normalized = aerotwin::normalizeQuaternion(scaled);
    ASSERT_TRUE(normalized);
    EXPECT_TRUE(aerotwin::inverseRotation(*normalized));
}

TEST(RotationsTest, UsesDefaultNormToleranceOnBothSidesAndRemovesAcceptedDrift) {
    for (double sign : {-1.0, 1.0}) {
        const Quaternion inside(1.0 + sign * 5e-13, 0.0, 0.0, 0.0);
        const Quaternion outside(1.0 + sign * 2e-12, 0.0, 0.0, 0.0);
        const auto inverse = aerotwin::inverseRotation(inside);
        const auto composed = aerotwin::composeRotations(inside, inside);
        const auto rotated = aerotwin::rotateVector(inside, Vector3::UnitX());
        ASSERT_TRUE(inverse);
        ASSERT_TRUE(composed);
        ASSERT_TRUE(rotated);
        EXPECT_DOUBLE_EQ(inverse->w(), 1.0);
        EXPECT_DOUBLE_EQ(composed->w(), 1.0);
        expectVectorNear(*rotated, Vector3::UnitX());
        EXPECT_FALSE(aerotwin::inverseRotation(outside));
        EXPECT_FALSE(aerotwin::rotateVector(outside, Vector3::UnitX()));
        EXPECT_FALSE(aerotwin::composeRotations(outside, inside));
        EXPECT_FALSE(aerotwin::composeRotations(inside, outside));
    }
    // Adjacent representable values straddle the fixed default upper boundary.
    const double upper = 1.0 + aerotwin::kQuaternionNormTolerance;
    EXPECT_FALSE(aerotwin::inverseRotation(Quaternion(upper, 0.0, 0.0, 0.0)));
    EXPECT_TRUE(aerotwin::inverseRotation(
        Quaternion(std::nextafter(upper, 1.0), 0.0, 0.0, 0.0)));
}

TEST(RotationsTest, RejectsNonFiniteVectorsAndUnrepresentableResults) {
    for (int axis = 0; axis < 3; ++axis) {
        for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
            Vector3 vector = Vector3::Zero();
            vector[axis] = invalid;
            EXPECT_FALSE(aerotwin::rotateVector(Quaternion::Identity(), vector));
        }
    }
    const double maximum = std::numeric_limits<double>::max();
    const auto large = aerotwin::rotateVector(quarterTurnZ(), Vector3(maximum, 0.0, 0.0));
    ASSERT_TRUE(large);
    EXPECT_NEAR(large->y() / maximum, 1.0, kTolerance);
    // A 45-degree Z rotation maps (max,max,0) to a Y component exceeding max.
    const Quaternion eighth_turn(std::cos(std::acos(-1.0) / 8.0), 0.0, 0.0,
                                std::sin(std::acos(-1.0) / 8.0));
    EXPECT_FALSE(aerotwin::rotateVector(eighth_turn, Vector3(maximum, maximum, 0.0)));
}

TEST(RotationsTest, IdentityPreservesMixedMagnitudeComponents) {
    const Vector3 vector(1e300, 1e-100, 0.0);
    const auto result = aerotwin::rotateVector(Quaternion::Identity(), vector);
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ(result->x(), vector.x());
    EXPECT_EQ(result->y(), vector.y()); // An absolute 1e-12 tolerance would miss total loss.
    EXPECT_DOUBLE_EQ(result->z(), vector.z());
}

TEST(RotationsTest, IdentityPreservesSubnormalComponents) {
    const double tiny = std::numeric_limits<double>::denorm_min();
    const Vector3 vector(1e300, tiny, -2.0 * tiny);
    const auto result = aerotwin::rotateVector(Quaternion::Identity(), vector);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->x(), vector.x());
    EXPECT_EQ(result->y(), tiny);
    EXPECT_EQ(result->z(), -2.0 * tiny);
}

TEST(RotationsTest, KnownRotationsPreserveUnmixedSmallComponents) {
    const Vector3 vector(1e300, 1e-100, -1e-200);
    // Exact 180-degree Z rotation: no cancellation between different components.
    const auto half_turn = aerotwin::rotateVector(Quaternion(0.0, 0.0, 0.0, 1.0), vector);
    ASSERT_TRUE(half_turn);
    EXPECT_EQ(half_turn->x(), -vector.x());
    EXPECT_EQ(half_turn->y(), -vector.y());
    EXPECT_EQ(half_turn->z(), vector.z());
    // Exact 120-degree rotation around (1,1,1): (x,y,z) -> (z,x,y).
    const auto cyclic = aerotwin::rotateVector(Quaternion(0.5, 0.5, 0.5, 0.5), vector);
    ASSERT_TRUE(cyclic);
    EXPECT_EQ(cyclic->x(), vector.z());
    EXPECT_EQ(cyclic->y(), vector.x());
    EXPECT_EQ(cyclic->z(), vector.y());
}

TEST(RotationsTest, NonIdentityRotationPreservesSubnormalPermutation) {
    const double tiny = std::numeric_limits<double>::denorm_min();
    const auto result = aerotwin::rotateVector(Quaternion(0.5, 0.5, 0.5, 0.5),
                                              Vector3(tiny, -2.0 * tiny, 3.0 * tiny));
    ASSERT_TRUE(result);
    EXPECT_EQ(result->x(), 3.0 * tiny);
    EXPECT_EQ(result->y(), tiny);
    EXPECT_EQ(result->z(), -2.0 * tiny);
}

TEST(RotationsTest, RecoversFiniteRowsAfterIntermediateOverflowAndCancellation) {
    const auto rotation = aerotwin::normalizeQuaternion(Quaternion(0.0, 1.0, 1.0, 1.0));
    ASSERT_TRUE(rotation);
    const double large = 0.9 * std::numeric_limits<double>::max();
    // Rotation by 180 degrees around (1,1,1) fixes this vector. In the last
    // matrix row, the two leading positive products sum above double's maximum
    // before the negative third product brings the final component back in range.
    const auto result = aerotwin::rotateVector(*rotation, Vector3(large, large, large));
    ASSERT_TRUE(result);
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR((*result)[axis] / large, 1.0, kTolerance);
    }
}

TEST(RotationsTest, NormalizesMixedMagnitudeQuaternionCoefficients) {
    const Quaternion input(1e300, 1e200, -1e100, 1.0);
    const auto result = aerotwin::normalizeQuaternion(input);
    ASSERT_TRUE(result);
    EXPECT_DOUBLE_EQ(result->w(), 1.0);
    EXPECT_NEAR(result->x() / 1e-100, 1.0, kTolerance);
    EXPECT_NEAR(result->y() / -1e-200, 1.0, kTolerance);
    EXPECT_NEAR(result->z() / 1e-300, 1.0, kTolerance);
    EXPECT_TRUE(aerotwin::isUnitQuaternion(*result));
}

TEST(RotationsTest, RemovesAcceptedDriftFromNonIdentityRotation) {
    const Quaternion exact(0.5, 0.5, 0.5, 0.5);
    for (double sign : {-1.0, 1.0}) {
        Quaternion drifted = exact;
        drifted.coeffs() *= 1.0 + sign * 5e-13;
        ASSERT_TRUE(aerotwin::isUnitQuaternion(drifted));
        const auto rotated = aerotwin::rotateVector(drifted, Vector3::UnitX());
        const auto inverse = aerotwin::inverseRotation(drifted);
        const auto composed = aerotwin::composeRotations(drifted, drifted);
        ASSERT_TRUE(rotated);
        ASSERT_TRUE(inverse);
        ASSERT_TRUE(composed);
        expectVectorNear(*rotated, Vector3::UnitY());
        EXPECT_TRUE(aerotwin::isUnitQuaternion(*inverse));
        EXPECT_TRUE(aerotwin::isUnitQuaternion(*composed));
        const auto recovered = aerotwin::rotateVector(*inverse, *rotated);
        const auto twice = aerotwin::rotateVector(*composed, Vector3::UnitX());
        ASSERT_TRUE(recovered);
        ASSERT_TRUE(twice);
        expectVectorNear(*recovered, Vector3::UnitX());
        expectVectorNear(*twice, Vector3::UnitZ());
    }
}

TEST(RotationsTest, RepeatedCompositionKeepsUnitNormAndExpectedRotation) {
    Quaternion accumulated = Quaternion::Identity();
    const Quaternion step = quarterTurnZ();
    for (int iteration = 0; iteration < 10000; ++iteration) {
        const auto next = aerotwin::composeRotations(step, accumulated);
        ASSERT_TRUE(next);
        accumulated = *next;
        EXPECT_NEAR(accumulated.coeffs().stableNorm(), 1.0, 1e-14);
    }
    const auto result = aerotwin::rotateVector(accumulated, Vector3::UnitX());
    ASSERT_TRUE(result);
    expectVectorNear(*result, Vector3::UnitX());
}

TEST(RotationsTest, RepeatedSmallAngleCompositionMatchesAccumulatedRotation) {
    constexpr int kIterations = 10000;
    constexpr double kStepAngle = 0.1; // radians
    // 1e-11 exceeds N*epsilon (~2.3e-12), allowing accumulated arithmetic and
    // trig argument-reduction error while detecting meaningful angular drift.
    constexpr double kAccumulatedTolerance = 1e-11;
    const Quaternion step(std::cos(kStepAngle / 2.0), 0.0, 0.0,
                          std::sin(kStepAngle / 2.0));
    Quaternion accumulated = Quaternion::Identity();
    for (int iteration = 0; iteration < kIterations; ++iteration) {
        const auto next = aerotwin::composeRotations(step, accumulated);
        ASSERT_TRUE(next);
        accumulated = *next;
        EXPECT_TRUE(accumulated.coeffs().allFinite());
        EXPECT_NEAR(accumulated.coeffs().stableNorm(), 1.0, 1e-14);
    }
    // Compare rotated basis vectors, avoiding quaternion sign ambiguity. Reduce
    // modulo 2*pi because orientation is periodic even when total angle is large.
    const double angle = std::remainder(kIterations * kStepAngle, 2.0 * std::acos(-1.0));
    const auto rotated = aerotwin::rotateVector(accumulated, Vector3::UnitX());
    ASSERT_TRUE(rotated);
    EXPECT_NEAR(rotated->x(), std::cos(angle), kAccumulatedTolerance);
    EXPECT_NEAR(rotated->y(), std::sin(angle), kAccumulatedTolerance);
    EXPECT_NEAR(rotated->z(), 0.0, kAccumulatedTolerance);
}
