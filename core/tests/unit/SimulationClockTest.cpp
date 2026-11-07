#include <aerotwin/simulation/SimulationClock.hpp>

#include <gtest/gtest.h>

#include <array>
#include <limits>

namespace aerotwin {
namespace {

template <typename Duration>
concept AcceptedClockTimestep = requires(Duration duration) {
    SimulationClock::create(duration);
};

static_assert(AcceptedClockTimestep<SimulationNanoseconds>);
static_assert(!AcceptedClockTimestep<std::chrono::seconds>);

} // namespace

TEST(SimulationClockTest, StartsAtZeroAndAdvancesByExactFixedSteps) {
    constexpr SimulationNanoseconds timestep{10'000'001};
    auto clock = SimulationClock::create(timestep);
    ASSERT_TRUE(clock);
    EXPECT_EQ(clock->now(), SimulationTimestamp{});
    EXPECT_EQ(clock->timestep(), timestep);
    EXPECT_FALSE(clock->isPaused());
    for (std::int64_t i = 1; i <= 100; ++i) {
        const auto previous = clock->now();
        ASSERT_TRUE(clock->step());
        EXPECT_EQ(clock->now().epoch, previous.epoch);
        EXPECT_GT(clock->now().elapsed, previous.elapsed);
        EXPECT_EQ(clock->now().elapsed.count(), i * timestep.count());
    }
}

TEST(SimulationClockTest, PauseAndResumeAreIdempotent) {
    auto clock = SimulationClock::create(SimulationNanoseconds{7});
    ASSERT_TRUE(clock);
    ASSERT_TRUE(clock->step());
    const auto before = clock->now();
    clock->pause();
    clock->pause();
    EXPECT_TRUE(clock->isPaused());
    EXPECT_FALSE(clock->step());
    EXPECT_EQ(clock->now(), before);
    clock->resume();
    clock->resume();
    EXPECT_FALSE(clock->isPaused());
    EXPECT_EQ(clock->now(), before);
    EXPECT_TRUE(clock->step());
    EXPECT_EQ(clock->now().epoch, before.epoch);
    EXPECT_EQ(clock->now().elapsed.count(), 14);
}

TEST(SimulationClockTest, ResetStartsNewEpochAtZeroAndUnpauses) {
    auto clock = SimulationClock::create(SimulationNanoseconds{9});
    ASSERT_TRUE(clock);
    const auto initial = clock->now();
    ASSERT_TRUE(clock->step());
    clock->pause();
    ASSERT_TRUE(clock->reset());
    EXPECT_EQ(clock->now().elapsed, initial.elapsed);
    EXPECT_NE(clock->now(), initial);
    EXPECT_EQ(clock->now().epoch, 1u);
    EXPECT_FALSE(clock->isPaused());
    EXPECT_EQ(clock->timestep().count(), 9);
    ASSERT_TRUE(clock->reset());
    EXPECT_EQ(clock->now().epoch, 2u);
    EXPECT_TRUE(clock->step());
    EXPECT_EQ(clock->now().elapsed.count(), 9);
}

TEST(SimulationClockTest, IdenticalOperationsProduceIdenticalTimestamps) {
    constexpr auto replay = [](SimulationClock& clock) {
        const auto initial = clock.now();
        EXPECT_TRUE(clock.step());
        const auto stepped = clock.now();
        clock.pause();
        EXPECT_FALSE(clock.step());
        const auto paused = clock.now();
        clock.resume();
        EXPECT_TRUE(clock.step());
        const auto resumed = clock.now();
        EXPECT_TRUE(clock.reset());
        return std::array{initial, stepped, paused, resumed, clock.now()};
    };
    auto first = SimulationClock::create(SimulationNanoseconds{123});
    auto second = SimulationClock::create(SimulationNanoseconds{123});
    ASSERT_TRUE(first && second);
    const auto expected = replay(*first);
    EXPECT_EQ(replay(*second), expected);
}

TEST(SimulationClockTest, RejectsNonpositiveTimesteps) {
    EXPECT_FALSE(SimulationClock::create(SimulationNanoseconds{0}));
    EXPECT_FALSE(SimulationClock::create(SimulationNanoseconds{-1}));
    EXPECT_FALSE(SimulationClock::create(SimulationNanoseconds::min()));
}

TEST(SimulationClockTest, CanReachTimestampLimitAndRejectsOverflowWithoutMutation) {
    auto clock = SimulationClock::create(SimulationNanoseconds::max());
    ASSERT_TRUE(clock);
    ASSERT_TRUE(clock->step());
    const auto at_limit = clock->now();
    EXPECT_EQ(at_limit.elapsed, SimulationNanoseconds::max());
    EXPECT_FALSE(clock->step());
    EXPECT_EQ(clock->now(), at_limit);
    EXPECT_FALSE(clock->isPaused());
    EXPECT_EQ(clock->timestep(), SimulationNanoseconds::max());
    ASSERT_TRUE(clock->reset());
    EXPECT_TRUE(clock->step());
}

class SimulationClockBoundaryTest : public testing::Test {
protected:
    static void placeBeforeFinalEpoch(SimulationClock& clock) {
        clock.timestamp_.epoch = std::numeric_limits<std::uint64_t>::max() - 1;
    }
};

TEST_F(SimulationClockBoundaryTest, CanReachEpochLimitAndRejectsResetWithoutMutation) {
    auto clock = SimulationClock::create(SimulationNanoseconds{5});
    ASSERT_TRUE(clock);
    placeBeforeFinalEpoch(*clock);
    ASSERT_TRUE(clock->reset());
    EXPECT_EQ(clock->now().epoch, std::numeric_limits<std::uint64_t>::max());
    ASSERT_TRUE(clock->step());
    clock->pause();
    const auto at_limit = clock->now();
    EXPECT_FALSE(clock->reset());
    EXPECT_EQ(clock->now(), at_limit);
    EXPECT_TRUE(clock->isPaused());
    EXPECT_EQ(clock->timestep().count(), 5);
}

} // namespace aerotwin
