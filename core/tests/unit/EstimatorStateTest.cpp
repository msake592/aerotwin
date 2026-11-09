#include <aerotwin/estimation/EstimatorState.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>
#include <utility>

namespace aerotwin {
namespace {

using StateResult = std::variant<EstimatorState, EstimatorStateError>;
static_assert(!std::is_default_constructible_v<EstimatorState>);
static_assert(std::is_same_v<decltype(std::declval<const EstimatorState&>().covariance()),
                             EstimatorCovariance6>);
static_assert(std::is_same_v<decltype(std::declval<const EstimatorState&>().vector()),
                             EstimatorVector6>);

StateResult createWithCovariance(const EstimatorCovariance6& covariance) {
    return EstimatorState::create(EstimatorVector6::Zero(), covariance, {});
}

void expectError(const StateResult& result, EstimatorStateError error) {
    ASSERT_TRUE(std::holds_alternative<EstimatorStateError>(result));
    EXPECT_EQ(std::get<EstimatorStateError>(result), error);
}

TEST(EstimatorStateTest, PreservesEnuOrderingUnitsAndExactTimestamp) {
    EstimatorVector6 input;
    input << 1, 2, 3, -4, -5, -6;
    const SimulationTimestamp timestamp{std::numeric_limits<std::uint64_t>::max(),
                                       SimulationNanoseconds::max()};
    const auto result = EstimatorState::create(input, EstimatorCovariance6::Identity(), timestamp);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& state = std::get<EstimatorState>(result);
    EXPECT_EQ(state.vector(), input);
    EXPECT_EQ(state.positionWorldM(), Vector3(1, 2, 3));
    EXPECT_EQ(state.velocityWorldMps(), Vector3(-4, -5, -6));
    EXPECT_EQ(state.timestamp(), timestamp);
}

TEST(EstimatorStateTest, OwnsIndependentSnapshotsAcrossCopiesAndMoves) {
    EstimatorVector6 input = EstimatorVector6::Ones();
    EstimatorCovariance6 covariance = EstimatorCovariance6::Identity();
    auto result = EstimatorState::create(input, covariance, {});
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    const auto& original = std::get<EstimatorState>(result);
    auto copy = original;
    const auto moved = std::move(copy);
    input.setZero();
    covariance.setZero();
    auto retrieved = original.vector();
    retrieved.setZero();
    auto retrieved_covariance = original.covariance();
    retrieved_covariance.setZero();
    EXPECT_EQ(original.vector(), EstimatorVector6::Ones());
    EXPECT_EQ(original.covariance(), EstimatorCovariance6::Identity());
    EXPECT_EQ(moved.vector(), original.vector());
    EXPECT_EQ(moved.covariance(), original.covariance());
    EXPECT_EQ(moved.timestamp(), original.timestamp());
}

TEST(EstimatorStateTest, RejectsNonfiniteStateAndNegativeElapsedTime) {
    EstimatorVector6 input = EstimatorVector6::Zero();
    const EstimatorCovariance6 covariance = EstimatorCovariance6::Identity();
    expectError(EstimatorState::create(input, covariance, {5, SimulationNanoseconds{-1}}),
                EstimatorStateError::InvalidTimestamp);
    input(0) = std::numeric_limits<double>::quiet_NaN();
    expectError(EstimatorState::create(input, covariance, {}), EstimatorStateError::InvalidState);
    input(0) = 0.0;
    input(5) = std::numeric_limits<double>::infinity();
    expectError(EstimatorState::create(input, covariance, {}), EstimatorStateError::InvalidState);
}

TEST(EstimatorStateTest, AcceptsCorrelatedMixedScalePositiveDefiniteCovariance) {
    EstimatorCovariance6 covariance = EstimatorCovariance6::Identity();
    covariance(0, 0) = 1e280;
    covariance(5, 5) = 1e-280;
    covariance(0, 5) = covariance(5, 0) = -0.5;
    const auto result = createWithCovariance(covariance);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    EXPECT_EQ(std::get<EstimatorState>(result).covariance(), covariance);
    covariance(0, 5) = covariance(5, 0) = -1.01;
    expectError(createWithCovariance(covariance), EstimatorStateError::InvalidCovariance);
}

TEST(EstimatorStateTest, AcceptsSingularAndZeroCovariance) {
    EstimatorCovariance6 covariance = EstimatorCovariance6::Zero();
    const auto zero = createWithCovariance(covariance);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(zero));
    EXPECT_EQ(std::get<EstimatorState>(zero).covariance(), covariance);
    covariance.block<2, 2>(2, 2).setOnes();
    const auto singular = createWithCovariance(covariance);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(singular));
    EXPECT_EQ(std::get<EstimatorState>(singular).covariance(), covariance);
    covariance(0, 2) = covariance(2, 0) = std::numeric_limits<double>::denorm_min();
    expectError(createWithCovariance(covariance), EstimatorStateError::InvalidCovariance);
}

TEST(EstimatorStateTest, CanonicalizesSmallAsymmetryAndRejectsLargerAsymmetry) {
    EstimatorCovariance6 covariance = EstimatorCovariance6::Identity();
    covariance(0, 5) = 4e-13;
    const auto result = createWithCovariance(covariance);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    covariance(0, 5) = covariance(5, 0) = 2e-13;
    EXPECT_EQ(std::get<EstimatorState>(result).covariance(), covariance);
    covariance(5, 0) = 2e-12;
    expectError(createWithCovariance(covariance), EstimatorStateError::InvalidCovariance);
}

TEST(EstimatorStateTest, ChecksFullSpectrumAndPreservesToleratedNearNullDirection) {
    EstimatorCovariance6 covariance = EstimatorCovariance6::Identity();
    // Every pair passes the correlation bound, but the six-axis mode is indefinite.
    covariance.setConstant(-0.21);
    covariance.diagonal().setOnes();
    expectError(createWithCovariance(covariance), EstimatorStateError::InvalidCovariance);
    covariance.setConstant(-0.2 - 5e-14);
    covariance.diagonal().setOnes();
    const auto near_null = createWithCovariance(covariance);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(near_null));
    EXPECT_EQ(std::get<EstimatorState>(near_null).covariance(), covariance);
}

TEST(EstimatorStateTest, HandlesExtremeFiniteAndRejectsInvalidCovarianceEntries) {
    constexpr double largest = std::numeric_limits<double>::max();
    constexpr double smallest = std::numeric_limits<double>::denorm_min();
    EstimatorCovariance6 covariance = EstimatorCovariance6::Zero();
    covariance.block<2, 2>(0, 0).setConstant(largest);
    covariance.block<2, 2>(4, 4).setConstant(smallest);
    const auto result = createWithCovariance(covariance);
    ASSERT_TRUE(std::holds_alternative<EstimatorState>(result));
    EXPECT_EQ(std::get<EstimatorState>(result).covariance(), covariance);
    covariance(3, 3) = -smallest;
    expectError(createWithCovariance(covariance), EstimatorStateError::InvalidCovariance);
    covariance(3, 3) = 0.0;
    covariance(0, 5) = std::numeric_limits<double>::quiet_NaN();
    expectError(createWithCovariance(covariance), EstimatorStateError::InvalidCovariance);
    covariance(0, 5) = std::numeric_limits<double>::infinity();
    expectError(createWithCovariance(covariance), EstimatorStateError::InvalidCovariance);
}

} // namespace
} // namespace aerotwin
