#include <aerotwin/simulation/ScenarioRunner.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace aerotwin {
namespace {

const Vector3 zero = Vector3::Zero();
const Quaternion identity = Quaternion::Identity();
constexpr SimulationNanoseconds dt{250'000'000};

TEST(ScenarioRunnerTest, ValidatesTimestepStepCountAndDurationBeforeAllocation) {
    const auto state = VehicleState::create(zero, identity, zero, zero, {});
    ASSERT_TRUE(state);

    EXPECT_FALSE(ScenarioConfig::create(*state, SimulationNanoseconds{-1}, 1, 0));
    EXPECT_FALSE(ScenarioConfig::create(*state, SimulationNanoseconds{0}, 1, 0));
    EXPECT_FALSE(ScenarioConfig::create(*state, SimulationNanoseconds{0}, 0, 0));
    EXPECT_FALSE(ScenarioConfig::create(*state, dt, -1, 0));
    EXPECT_FALSE(ScenarioConfig::create(*state, dt, ScenarioConfig::kMaxSteps + 1, 0));
    EXPECT_FALSE(ScenarioConfig::create(*state, dt, std::numeric_limits<std::int64_t>::max(), 0));
    EXPECT_FALSE(ScenarioConfig::create(*state, SimulationNanoseconds::max(), 2, 0));
    EXPECT_TRUE(ScenarioConfig::create(*state, SimulationNanoseconds::max(), 1, 0));
    EXPECT_TRUE(ScenarioConfig::create(*state, dt, ScenarioConfig::kMaxSteps, 0));
}

TEST(ScenarioRunnerTest, RequiresFreshClockEpochAndElapsedTime) {
    for (const auto timestamp : {SimulationTimestamp{1, {}},
                                 SimulationTimestamp{0, SimulationNanoseconds{1}}}) {
        const auto state = VehicleState::create(zero, identity, zero, zero, timestamp);
        ASSERT_TRUE(state);
        EXPECT_FALSE(ScenarioConfig::create(*state, dt, 0, 0));
        EXPECT_EQ(state->timestamp(), timestamp);
    }
}

TEST(ScenarioRunnerTest, ValidatesAndOwnsOptionalIdentifier) {
    const auto state = VehicleState::create(zero, identity, zero, zero, {});
    ASSERT_TRUE(state);
    EXPECT_FALSE(ScenarioConfig::create(*state, dt, 1, 0, ""));
    EXPECT_FALSE(ScenarioConfig::create(*state, dt, 1, 0, "two words"));
    EXPECT_FALSE(ScenarioConfig::create(*state, dt, 1, 0, "x\nseed=3"));
    EXPECT_FALSE(ScenarioConfig::create(*state, dt, 1, 0, std::string(65, 'x')));

    std::string identifier = "run-1.ENu";
    const auto config = ScenarioConfig::create(*state, dt, 1, 42, identifier);
    ASSERT_TRUE(config);
    identifier.clear();
    EXPECT_EQ(config->identifier(), "run-1.ENu");
    EXPECT_EQ(config->seed(), 42U);
    EXPECT_EQ(config->timestep(), dt);
    EXPECT_EQ(config->stepCount(), 1);
}

TEST(ScenarioRunnerTest, RecordsInitialSampleAndExactClockSteps) {
    const Vector3 p(4.0, -3.0, 2.0);
    const Vector3 v(-2.0, 4.0, 1.0);
    const Vector3 w(1.0, 2.0, 3.0);
    const double s = std::sqrt(0.5);
    const Quaternion q(s, s, 0.0, 0.0);
    const auto state = VehicleState::create(p, q, v, w, {});
    ASSERT_TRUE(state);

    for (const auto steps : {0, 1, 4}) {
        const auto config = ScenarioConfig::create(*state, dt, steps, 42);
        ASSERT_TRUE(config);
        const auto result = runScenario(*config);
        ASSERT_TRUE(result);
        ASSERT_EQ(result->samples().size(), static_cast<std::size_t>(steps + 1));
        EXPECT_EQ(result->samples().front().orientationWorldFromBody().coeffs(), q.coeffs());
        for (int i = 0; i <= steps; ++i) {
            const auto& sample = result->samples()[i];
            const SimulationTimestamp expected{0, i * dt};
            EXPECT_EQ(sample.timestamp(), expected);
            EXPECT_EQ(sample.positionWorldM(), p + (0.25 * i) * v);
            EXPECT_EQ(sample.linearVelocityWorldMps(), v);
            EXPECT_EQ(sample.angularVelocityBodyRadps(), w);
        }
        EXPECT_EQ(config->initialState().timestamp(), SimulationTimestamp{});
        EXPECT_EQ(config->initialState().positionWorldM(), p);
        EXPECT_EQ(config->initialState().orientationWorldFromBody().coeffs(), q.coeffs());
    }
}

TEST(ScenarioRunnerTest, ReplaysExactlyAndSeedDoesNotAffectTruth) {
    const auto state = VehicleState::create(Vector3::Ones(), identity, Vector3::UnitY(),
                                            Vector3::Ones(), {});
    ASSERT_TRUE(state);
    const auto first_config = ScenarioConfig::create(*state, dt, 4, 42);
    const auto other_config = ScenarioConfig::create(*state, dt, 4, 43);
    ASSERT_TRUE(first_config);
    ASSERT_TRUE(other_config);
    const auto first = runScenario(*first_config);
    const auto repeat = runScenario(*first_config);
    const auto other_seed = runScenario(*other_config);
    ASSERT_TRUE(first);
    ASSERT_TRUE(repeat);
    ASSERT_TRUE(other_seed);
    EXPECT_EQ(other_seed->config().seed(), 43U);
    for (const auto* other : {&*repeat, &*other_seed}) {
        ASSERT_EQ(first->samples().size(), other->samples().size());
        for (std::size_t i = 0; i < first->samples().size(); ++i) {
            const auto& a = first->samples()[i];
            const auto& b = other->samples()[i];
            EXPECT_EQ(a.timestamp(), b.timestamp());
            EXPECT_EQ(a.positionWorldM(), b.positionWorldM());
            EXPECT_EQ(a.orientationWorldFromBody().coeffs(), b.orientationWorldFromBody().coeffs());
            EXPECT_EQ(a.linearVelocityWorldMps(), b.linearVelocityWorldMps());
            EXPECT_EQ(a.angularVelocityBodyRadps(), b.angularVelocityBodyRadps());
        }
    }
}

TEST(ScenarioRunnerTest, DiscardsPartialTrajectoryOnIntegrationFailure) {
    const Vector3 v(std::numeric_limits<double>::max(), 0.0, 0.0);
    const auto state = VehicleState::create(zero, identity, v, zero, {});
    ASSERT_TRUE(state);
    const auto config = ScenarioConfig::create(*state, SimulationNanoseconds{1'000'000'000}, 2, 0);
    ASSERT_TRUE(config);
    EXPECT_FALSE(runScenario(*config));
    EXPECT_EQ(config->initialState().timestamp(), SimulationTimestamp{});
    EXPECT_EQ(config->initialState().positionWorldM(), zero);
    EXPECT_EQ(config->initialState().linearVelocityWorldMps(), v);
}

TEST(ScenarioRunnerTest, ExecutesMaximumStepCount) {
    const Vector3 position(1.0, -2.0, 3.0);
    const auto state = VehicleState::create(position, identity, zero, zero, {});
    ASSERT_TRUE(state);
    constexpr auto steps = ScenarioConfig::kMaxSteps;
    const auto config = ScenarioConfig::create(*state, dt, steps, 0);
    ASSERT_TRUE(config);
    const auto result = runScenario(*config);
    ASSERT_TRUE(result);
    ASSERT_EQ(result->samples().size(), static_cast<std::size_t>(steps + 1));
    const auto& final = result->samples().back();
    const SimulationTimestamp expected{0, steps * dt};
    EXPECT_EQ(final.timestamp(), expected);
    EXPECT_EQ(final.positionWorldM(), position);
    EXPECT_EQ(final.orientationWorldFromBody().coeffs(), identity.coeffs());
}

} // namespace
} // namespace aerotwin
