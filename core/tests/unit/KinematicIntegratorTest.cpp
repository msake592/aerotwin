#include <aerotwin/math/Rotations.hpp>
#include <aerotwin/simulation/KinematicIntegrator.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

namespace aerotwin {
namespace {

constexpr SimulationTimestamp start{4, SimulationNanoseconds{7}};
constexpr SimulationTimestamp one_second_later{4, SimulationNanoseconds{1'000'000'007}};
const Vector3 zero = Vector3::Zero();
const Quaternion identity = Quaternion::Identity();

TEST(KinematicIntegratorTest, StationaryStatePreservesPositionAndOrientationCoefficients) {
    const Vector3 position(1.0, -2.0, 3.0);
    const Quaternion attitude(1.0 + 0.5 * kQuaternionNormTolerance, 0.0, 0.0, 0.0);
    const auto initial = VehicleState::create(position, attitude, zero, zero, start);
    ASSERT_TRUE(initial);

    const auto next = integrateConstantVelocity(*initial, one_second_later);
    ASSERT_TRUE(next);
    EXPECT_EQ(next->positionWorldM(), position);
    EXPECT_EQ(next->orientationWorldFromBody().coeffs(), attitude.coeffs());
    EXPECT_EQ(next->linearVelocityWorldMps(), zero);
    EXPECT_EQ(next->angularVelocityBodyRadps(), zero);
    EXPECT_EQ(next->timestamp(), one_second_later);
}

TEST(KinematicIntegratorTest, TranslatesInWorldAndPreservesBothVelocitiesAndInput) {
    const Vector3 position(1.0, 2.0, 3.0);
    const Vector3 linear_velocity(-2.0, 4.0, -6.0);
    const Vector3 angular_velocity(0.0, 0.0, std::numbers::pi / 2.0);
    const auto initial =
        VehicleState::create(position, identity, linear_velocity, angular_velocity, start);
    ASSERT_TRUE(initial);

    const auto next = integrateConstantVelocity(*initial, one_second_later);
    ASSERT_TRUE(next);
    EXPECT_EQ(next->positionWorldM(), Vector3(-1.0, 6.0, -3.0));
    EXPECT_EQ(next->linearVelocityWorldMps(), linear_velocity);
    EXPECT_EQ(next->angularVelocityBodyRadps(), angular_velocity);
    EXPECT_EQ(next->timestamp(), one_second_later);
    const auto rotated = rotateVector(next->orientationWorldFromBody(), Vector3::UnitX());
    ASSERT_TRUE(rotated);
    EXPECT_NEAR((*rotated - Vector3::UnitY()).norm(), 0.0, 1e-12);

    EXPECT_EQ(initial->positionWorldM(), position);
    EXPECT_EQ(initial->orientationWorldFromBody().coeffs(), identity.coeffs());
    EXPECT_EQ(initial->linearVelocityWorldMps(), linear_velocity);
    EXPECT_EQ(initial->angularVelocityBodyRadps(), angular_velocity);
    EXPECT_EQ(initial->timestamp(), start);
}

TEST(KinematicIntegratorTest, ComposesBodyIncrementOnTheRight) {
    const double s = std::sqrt(0.5);
    const Quaternion world_from_body(s, s, 0.0, 0.0);
    const Vector3 angular_velocity(0.0, 0.0, std::numbers::pi / 2.0);
    const auto initial = VehicleState::create(zero, world_from_body, zero, angular_velocity, start);
    ASSERT_TRUE(initial);

    const auto next = integrateConstantVelocity(*initial, one_second_later);
    ASSERT_TRUE(next);
    const auto rotated = rotateVector(next->orientationWorldFromBody(), Vector3::UnitX());
    ASSERT_TRUE(rotated);
    // Body Z rotation sends X to Y, then initial world-from-body sends Y to Z.
    EXPECT_NEAR((*rotated - Vector3::UnitZ()).norm(), 0.0, 1e-12);
}

TEST(KinematicIntegratorTest, EqualTimestampReturnsExactSnapshot) {
    const Vector3 position(1.0, 2.0, 3.0);
    const Vector3 linear_velocity(4.0, 5.0, 6.0);
    const Vector3 angular_velocity(7.0, 8.0, 9.0);
    const Quaternion attitude(1.0 + 0.5 * kQuaternionNormTolerance, 0.0, 0.0, 0.0);
    const auto initial =
        VehicleState::create(position, attitude, linear_velocity, angular_velocity, start);
    ASSERT_TRUE(initial);

    const auto next = integrateConstantVelocity(*initial, start);
    ASSERT_TRUE(next);
    EXPECT_EQ(next->positionWorldM(), position);
    EXPECT_EQ(next->orientationWorldFromBody().coeffs(), attitude.coeffs());
    EXPECT_EQ(next->linearVelocityWorldMps(), linear_velocity);
    EXPECT_EQ(next->angularVelocityBodyRadps(), angular_velocity);
    EXPECT_EQ(next->timestamp(), start);
}

TEST(KinematicIntegratorTest, RejectsEarlierTimesAndDifferentEpochs) {
    const auto initial = VehicleState::create(zero, identity, zero, zero, start);
    ASSERT_TRUE(initial);
    EXPECT_FALSE(integrateConstantVelocity(*initial, {4, SimulationNanoseconds{6}}));
    EXPECT_FALSE(integrateConstantVelocity(*initial, {4, SimulationNanoseconds::min()}));
    EXPECT_FALSE(integrateConstantVelocity(*initial, {3, start.elapsed}));
    EXPECT_FALSE(integrateConstantVelocity(*initial, {5, start.elapsed}));
}

TEST(KinematicIntegratorTest, PreservesTinyAngularIncrement) {
    const auto initial = VehicleState::create(zero, identity, zero, Vector3(1e-300, 0.0, 0.0), start);
    ASSERT_TRUE(initial);

    const auto next = integrateConstantVelocity(*initial, one_second_later);
    ASSERT_TRUE(next);
    EXPECT_NEAR(next->orientationWorldFromBody().x(), 5e-301, 1e-315);
    EXPECT_TRUE(isUnitQuaternion(next->orientationWorldFromBody()));
}

TEST(KinematicIntegratorTest, RepeatedStepsAreDeterministic) {
    const Vector3 angular_velocity(0.0, 0.0, std::numbers::pi / 2.0);
    const auto initial = VehicleState::create(zero, identity, Vector3::UnitX(), angular_velocity,
                                              start);
    ASSERT_TRUE(initial);

    auto first = initial;
    auto second = initial;
    for (int step = 1; step <= 4; ++step) {
        const SimulationTimestamp target{start.epoch, start.elapsed + SimulationNanoseconds{
                                                                         step * 250'000'000}};
        first = integrateConstantVelocity(*first, target);
        second = integrateConstantVelocity(*second, target);
        ASSERT_TRUE(first);
        ASSERT_TRUE(second);
        EXPECT_EQ(first->timestamp(), target);
        EXPECT_EQ(first->positionWorldM(), second->positionWorldM());
        EXPECT_EQ(first->orientationWorldFromBody().coeffs(),
                  second->orientationWorldFromBody().coeffs());
    }
    EXPECT_EQ(first->positionWorldM(), Vector3::UnitX());
    const auto rotated = rotateVector(first->orientationWorldFromBody(), Vector3::UnitX());
    ASSERT_TRUE(rotated);
    EXPECT_NEAR((*rotated - Vector3::UnitY()).norm(), 0.0, 1e-12);
}

TEST(KinematicIntegratorTest, SubtractsIntegerTimesBeforeConvertingToSeconds) {
    const SimulationTimestamp target{std::numeric_limits<std::uint64_t>::max(),
                                     SimulationNanoseconds::max()};
    const SimulationTimestamp near_limit{target.epoch, target.elapsed - SimulationNanoseconds{1}};
    const auto initial = VehicleState::create(zero, identity, Vector3(1e9, 0.0, 0.0), zero,
                                              near_limit);
    ASSERT_TRUE(initial);
    const auto next = integrateConstantVelocity(*initial, target);
    ASSERT_TRUE(next);
    EXPECT_DOUBLE_EQ(next->positionWorldM().x(), 1.0);
    EXPECT_EQ(next->timestamp(), target);

    const auto at_zero = VehicleState::create(zero, identity, Vector3::UnitX(), zero,
                                              {target.epoch, SimulationNanoseconds::zero()});
    ASSERT_TRUE(at_zero);
    const auto full_span = integrateConstantVelocity(*at_zero, target);
    ASSERT_TRUE(full_span);
    EXPECT_DOUBLE_EQ(full_span->positionWorldM().x(), 9.223372036854776e9);
    EXPECT_EQ(full_span->timestamp(), target);
}

TEST(KinematicIntegratorTest, RejectsUnrepresentablePositionOrAngularIncrement) {
    const double max = std::numeric_limits<double>::max();
    const auto fast = VehicleState::create(zero, identity, Vector3(max, 0.0, 0.0), zero, start);
    const auto spinning = VehicleState::create(zero, identity, zero, Vector3::Constant(max), start);
    ASSERT_TRUE(fast);
    ASSERT_TRUE(spinning);

    const SimulationTimestamp two_seconds_later{4, SimulationNanoseconds{2'000'000'007}};
    EXPECT_FALSE(integrateConstantVelocity(*fast, two_seconds_later));
    EXPECT_FALSE(integrateConstantVelocity(*spinning, two_seconds_later));
    EXPECT_EQ(fast->positionWorldM(), zero);
    EXPECT_EQ(spinning->timestamp(), start);
}

TEST(KinematicIntegratorTest, FusedPositionUpdatePreservesFiniteCancellation) {
    const double max = std::numeric_limits<double>::max();
    const auto initial = VehicleState::create(Vector3(-max, 0.0, 0.0), identity,
                                              Vector3(max, 0.0, 0.0), zero, start);
    ASSERT_TRUE(initial);
    const SimulationTimestamp two_seconds_later{4, SimulationNanoseconds{2'000'000'007}};
    const auto next = integrateConstantVelocity(*initial, two_seconds_later);
    ASSERT_TRUE(next);
    EXPECT_EQ(next->positionWorldM(), Vector3(max, 0.0, 0.0));
}

} // namespace
} // namespace aerotwin
