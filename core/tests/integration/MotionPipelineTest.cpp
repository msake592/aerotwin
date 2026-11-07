#include <aerotwin/math/Transformations.hpp>
#include <aerotwin/telemetry/TruthTelemetry.hpp>

#include "../support/TemporaryDirectory.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iterator>
#include <numbers>
#include <sstream>

namespace aerotwin {
namespace {

constexpr std::int64_t steps = 100;
constexpr SimulationNanoseconds timestep{100'000'000};
constexpr double position_tolerance_m = 1e-12;
constexpr double orientation_tolerance = 1e-12;

struct MotionCase {
    bool translates;
    bool rotates;
    const char* name;
};

class MotionPipelineTest : public testing::TestWithParam<MotionCase> {};

TEST_P(MotionPipelineTest, MatchesAnalyticalTrajectory) {
    const auto motion = GetParam();
    const Vector3 p0(10.0, -5.0, 2.0);
    const Vector3 v = motion.translates ? Vector3(2.0, 1.0, 0.5) : Vector3(0.0, 0.0, 0.0);
    const double rate = motion.rotates ? std::numbers::pi / 4.0 : 0.0;
    const Vector3 w(0.0, 0.0, rate);
    const auto initial = VehicleState::create(p0, Quaternion::Identity(), v, w, {});
    ASSERT_TRUE(initial);
    const auto config = ScenarioConfig::create(*initial, timestep, steps, 42, motion.name);
    ASSERT_TRUE(config);
    const auto result = runScenario(*config);
    ASSERT_TRUE(result);
    ASSERT_EQ(result->samples().size(), steps + 1);

    for (std::int64_t i = 0; i <= steps; ++i) {
        SCOPED_TRACE(i);
        const auto& sample = result->samples()[i];
        const SimulationTimestamp timestamp{0, i * timestep};
        EXPECT_EQ(sample.timestamp(), timestamp);
        EXPECT_EQ(sample.linearVelocityWorldMps(), v);
        EXPECT_EQ(sample.angularVelocityBodyRadps(), w);

        const double t = static_cast<double>(i) / 10.0;
        const Vector3 expected_position = p0 + t * v;
        EXPECT_LE((sample.positionWorldM() - expected_position).norm(), position_tolerance_m);

        const double half_angle = 0.5 * rate * t;
        const Quaternion expected(std::cos(half_angle), 0.0, 0.0, std::sin(half_angle));
        const auto actual = sample.orientationWorldFromBody();
        EXPECT_TRUE(isUnitQuaternion(actual));
        EXPECT_LE(std::min((actual.coeffs() - expected.coeffs()).norm(),
                           (actual.coeffs() + expected.coeffs()).norm()),
                  orientation_tolerance);
    }

    if (motion.translates && motion.rotates) {
        const auto& last = result->samples().back();
        const auto forward = bodyToWorldVector(last.orientationWorldFromBody(), Vector3::UnitX());
        ASSERT_TRUE(forward);
        EXPECT_LE((*forward - Vector3::UnitY()).norm(), orientation_tolerance);

        const test::TemporaryDirectory temporary;
        const auto first = temporary.path() / "first";
        const auto second = temporary.path() / "second";
        const auto replay = runScenario(*config);
        ASSERT_TRUE(replay);
        ASSERT_EQ(replay->samples().size(), result->samples().size());
        ASSERT_TRUE(exportTruthTelemetry(*result, first));
        ASSERT_TRUE(exportTruthTelemetry(*replay, second));

        std::ifstream first_csv(first / "truth_trajectory.csv", std::ios::binary);
        std::ifstream second_csv(second / "truth_trajectory.csv", std::ios::binary);
        ASSERT_TRUE(first_csv);
        ASSERT_TRUE(second_csv);
        const std::string bytes{std::istreambuf_iterator<char>(first_csv), {}};
        EXPECT_EQ(bytes, (std::string{std::istreambuf_iterator<char>(second_csv), {}}));

        std::istringstream rows(bytes);
        std::string row;
        ASSERT_TRUE(std::getline(rows, row));
        EXPECT_EQ(std::count(row.begin(), row.end(), ','), 14);
        for (const auto& sample : result->samples()) {
            ASSERT_TRUE(std::getline(rows, row));
            ASSERT_EQ(std::count(row.begin(), row.end(), ','), 14);
            std::replace(row.begin(), row.end(), ',', ' ');
            std::istringstream fields(row);
            SimulationTimestamp timestamp{};
            std::int64_t elapsed{};
            std::array<double, 13> values{};
            ASSERT_TRUE(fields >> timestamp.epoch >> elapsed);
            timestamp.elapsed = SimulationNanoseconds{elapsed};
            EXPECT_EQ(timestamp, sample.timestamp());
            for (auto& value : values) {
                ASSERT_TRUE(fields >> value);
            }
            const auto p = sample.positionWorldM();
            const auto q = sample.orientationWorldFromBody();
            const std::array expected{p.x(), p.y(), p.z(), q.w(), q.x(), q.y(), q.z(),
                                      v.x(), v.y(), v.z(), w.x(), w.y(), w.z()};
            EXPECT_EQ(values, expected);
        }
        EXPECT_FALSE(std::getline(rows, row));

        std::ifstream metadata(first / "run_metadata.txt");
        ASSERT_TRUE(metadata);
        const std::string text{std::istreambuf_iterator<char>(metadata), {}};
        EXPECT_NE(text.find("scenario_id=Combined\nseed=42\ntimestep_ns=100000000\n"
                            "step_count=100\nsample_count=101\nworld_frame=ENU\nbody_frame=FLU\n"),
                  std::string::npos);
    }
}

INSTANTIATE_TEST_SUITE_P(ReferenceMotions, MotionPipelineTest,
                        testing::Values(MotionCase{false, false, "Stationary"},
                                        MotionCase{true, false, "StraightLine"},
                                        MotionCase{false, true, "RotationInPlace"},
                                        MotionCase{true, true, "Combined"}),
                        [](const auto& info) { return info.param.name; });

} // namespace
} // namespace aerotwin
