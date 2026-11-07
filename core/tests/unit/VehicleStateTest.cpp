#include <aerotwin/math/Transformations.hpp>
#include <aerotwin/simulation/VehicleState.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <type_traits>

namespace aerotwin {
namespace {

static_assert(!std::is_default_constructible_v<VehicleState>);
static_assert(!std::is_constructible_v<VehicleState, Vector3, Quaternion, Vector3, Vector3,
                                       SimulationTimestamp>);

const Vector3 zero = Vector3::Zero();
const Quaternion identity = Quaternion::Identity();
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
constexpr double infinity = std::numeric_limits<double>::infinity();

TEST(VehicleStateTest, AcceptsExplicitZeroState) {
    const auto state = VehicleState::create(zero, identity, zero, zero, {});
    ASSERT_TRUE(state);
    EXPECT_EQ(state->positionWorldM(), zero);
    EXPECT_EQ(state->orientationWorldFromBody().coeffs(), identity.coeffs());
    EXPECT_EQ(state->linearVelocityWorldMps(), zero);
    EXPECT_EQ(state->angularVelocityBodyRadps(), zero);
    EXPECT_EQ(state->timestamp(), SimulationTimestamp{});
}

TEST(VehicleStateTest, PreservesFrameComponentsAndSimulationTimestamp) {
    Vector3 position(1.0, -2.0, 3.0);
    const Vector3 linear_velocity(-4.0, 5.0, 6.0);
    const Vector3 angular_velocity(0.1, -0.2, 0.3);
    const double half_sqrt_two = std::sqrt(0.5);
    const Quaternion attitude(half_sqrt_two, 0.0, 0.0, half_sqrt_two);
    const SimulationTimestamp timestamp{3, SimulationNanoseconds{42}};
    const auto state =
        VehicleState::create(position, attitude, linear_velocity, angular_velocity, timestamp);

    ASSERT_TRUE(state);
    EXPECT_EQ(position, Vector3(1.0, -2.0, 3.0));
    position.setZero();
    EXPECT_EQ(state->positionWorldM(), Vector3(1.0, -2.0, 3.0));
    EXPECT_EQ(state->linearVelocityWorldMps(), linear_velocity);
    EXPECT_EQ(state->angularVelocityBodyRadps(), angular_velocity);
    EXPECT_EQ(state->orientationWorldFromBody().coeffs(), attitude.coeffs());
    EXPECT_EQ(state->timestamp(), timestamp);

    const auto forward_in_world =
        bodyToWorldVector(state->orientationWorldFromBody(), Vector3::UnitX());
    ASSERT_TRUE(forward_in_world);
    EXPECT_TRUE(forward_in_world->isApprox(Vector3::UnitY()));
}

TEST(VehicleStateTest, PositionAccessorReturnsIndependentCopy) {
    const Vector3 position(1.0, -2.0, 3.0);
    const auto state = VehicleState::create(position, identity, zero, zero, {});
    ASSERT_TRUE(state);

    decltype(auto) position_copy = state->positionWorldM();
    position_copy.setZero();

    EXPECT_EQ(position_copy, zero);
    EXPECT_EQ(state->positionWorldM(), position);
}

TEST(VehicleStateTest, RejectsNonFinitePosition) {
    EXPECT_FALSE(VehicleState::create(Vector3(nan, 0.0, 0.0), identity, zero, zero, {}));
    EXPECT_FALSE(VehicleState::create(Vector3(0.0, infinity, 0.0), identity, zero, zero, {}));
}

TEST(VehicleStateTest, RejectsNonFiniteVelocities) {
    EXPECT_FALSE(VehicleState::create(zero, identity, Vector3(0.0, 0.0, nan), zero, {}));
    EXPECT_FALSE(VehicleState::create(zero, identity, zero, Vector3(0.0, -infinity, 0.0), {}));
}

TEST(VehicleStateTest, RejectsInvalidOrientations) {
    EXPECT_FALSE(VehicleState::create(zero, Quaternion(1.0, nan, 0.0, 0.0), zero, zero, {}));
    EXPECT_FALSE(VehicleState::create(zero, Quaternion(0.0, 0.0, 0.0, 0.0), zero, zero, {}));
    const Quaternion outside(1.0 + 2.0 * kQuaternionNormTolerance, 0.0, 0.0, 0.0);
    EXPECT_FALSE(VehicleState::create(zero, outside, zero, zero, {}));
}

TEST(VehicleStateTest, PreservesAcceptedQuaternionDrift) {
    const Quaternion drifted(1.0 + 0.5 * kQuaternionNormTolerance, 0.0, 0.0, 0.0);
    const auto state = VehicleState::create(zero, drifted, zero, zero, {});
    ASSERT_TRUE(state);
    EXPECT_EQ(state->orientationWorldFromBody().coeffs(), drifted.coeffs());
}

TEST(VehicleStateTest, RejectsNegativeElapsedTime) {
    EXPECT_FALSE(VehicleState::create(zero, identity, zero, zero, {3, SimulationNanoseconds{-1}}));
}

TEST(VehicleStateTest, AcceptsTimestampLimitsAndLargeFiniteComponents) {
    const Vector3 large = Vector3::Constant(std::numeric_limits<double>::max());
    const SimulationTimestamp timestamp{std::numeric_limits<std::uint64_t>::max(),
                                        SimulationNanoseconds::max()};
    const auto state = VehicleState::create(large, identity, large, large, timestamp);
    ASSERT_TRUE(state);
    EXPECT_EQ(state->timestamp(), timestamp);
}

} // namespace
} // namespace aerotwin
