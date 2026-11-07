#include <aerotwin/telemetry/TruthTelemetry.hpp>

#include "../../src/telemetry/TruthTelemetryInternal.hpp"
#include "../support/TemporaryDirectory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iterator>
#include <sstream>

namespace aerotwin {
namespace {

std::string readFile(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

TEST(TruthTelemetryTest, WritesOrderedRowsWithRoundTripPrecisionAndWxyzOrientation) {
    const double precise = std::nextafter(1.0, 2.0);
    const Quaternion attitude(0.5, -0.5, 0.5, -0.5);
    const Vector3 position(precise, -2.0, 3.0);
    const Vector3 velocity(0.0, 1.0, 0.0);
    const Vector3 zero = Vector3::Zero();
    const auto initial = VehicleState::create(position, attitude, velocity, zero, {});
    ASSERT_TRUE(initial);
    const auto config =
        ScenarioConfig::create(*initial, SimulationNanoseconds{1'000'000'000}, 1, 42, "export");
    ASSERT_TRUE(config);
    const auto result = runScenario(*config);
    const auto replay = runScenario(*config);
    ASSERT_TRUE(result);
    ASSERT_TRUE(replay);

    const test::TemporaryDirectory temporary;
    const auto output = temporary.path() / "first";
    const auto repeated = temporary.path() / "repeat";
    ASSERT_TRUE(exportTruthTelemetry(*result, output));
    ASSERT_TRUE(exportTruthTelemetry(*replay, repeated));

    const auto csv = readFile(output / "truth_trajectory.csv");
    EXPECT_EQ(csv, readFile(repeated / "truth_trajectory.csv"));
    std::istringstream rows(csv);
    std::string row;
    ASSERT_TRUE(std::getline(rows, row));
    EXPECT_EQ(row, "epoch,elapsed_ns,position_e_m,position_n_m,position_u_m,"
                   "orientation_w,orientation_x,orientation_y,orientation_z,"
                   "velocity_e_mps,velocity_n_mps,velocity_u_mps,"
                   "angular_velocity_f_radps,angular_velocity_l_radps,angular_velocity_u_radps");
    ASSERT_TRUE(std::getline(rows, row));
    EXPECT_EQ(row, "0,0,1.0000000000000002,-2,3,0.5,-0.5,0.5,-0.5,0,1,0,0,0,0");
    ASSERT_TRUE(std::getline(rows, row));
    EXPECT_EQ(row, "0,1000000000,1.0000000000000002,-1,3,0.5,-0.5,0.5,-0.5,0,1,0,0,0,0");
    EXPECT_FALSE(std::getline(rows, row));

    const auto metadata = readFile(output / "run_metadata.txt");
    EXPECT_EQ(metadata, readFile(repeated / "run_metadata.txt"));
    EXPECT_EQ(metadata, "schema_version=1\nscenario_id=export\nseed=42\n"
                        "timestep_ns=1000000000\nstep_count=1\nsample_count=2\n"
                        "world_frame=ENU\nbody_frame=FLU\n"
                        "orientation=active_Hamilton_body_to_world_WXYZ\n"
                        "time_unit=ns\nposition_unit=m\nlinear_velocity_unit=m/s\n"
                        "angular_velocity_unit=rad/s\n");
}

TEST(TruthTelemetryTest, RejectsInvalidPathsAndNeverOverwritesExistingOutput) {
    const Vector3 zero = Vector3::Zero();
    const auto state = VehicleState::create(zero, Quaternion::Identity(), zero, zero, {});
    ASSERT_TRUE(state);
    const auto config = ScenarioConfig::create(*state, SimulationNanoseconds{1}, 0, 0);
    ASSERT_TRUE(config);
    const auto result = runScenario(*config);
    ASSERT_TRUE(result);

    const test::TemporaryDirectory temporary;
    const auto existing = temporary.path() / "existing";
    ASSERT_TRUE(std::filesystem::create_directory(existing));
    const auto sentinel = existing / "truth_trajectory.csv";
    std::ofstream(sentinel) << "preserve";
    EXPECT_FALSE(exportTruthTelemetry(*result, {}));
    EXPECT_FALSE(exportTruthTelemetry(*result, sentinel));
    EXPECT_FALSE(exportTruthTelemetry(*result, sentinel / "invalid"));
    EXPECT_FALSE(exportTruthTelemetry(*result, temporary.path() / "missing" / "output"));
    EXPECT_FALSE(exportTruthTelemetry(*result, existing));
    EXPECT_EQ(readFile(sentinel), "preserve");
    EXPECT_FALSE(std::filesystem::exists(existing / "run_metadata.txt"));
    EXPECT_FALSE(std::filesystem::exists(temporary.path() / "missing"));
}

std::optional<ScenarioResult> stationaryResult() {
    const auto state = VehicleState::create(Vector3::Zero(), Quaternion::Identity(),
                                            Vector3::Zero(), Vector3::Zero(), {});
    const auto config = ScenarioConfig::create(*state, SimulationNanoseconds{1}, 0, 0);
    return runScenario(*config);
}

TEST(TruthTelemetryTest, RejectsSymlinkDestinationWithoutTouchingTarget) {
    const auto result = stationaryResult();
    ASSERT_TRUE(result);
    const test::TemporaryDirectory temporary;
    const auto target = temporary.path() / "unrelated";
    const auto link = temporary.path() / "output";
    ASSERT_TRUE(std::filesystem::create_directory(target));
    std::ofstream(target / "truth_trajectory.csv") << "original CSV";
    std::ofstream(target / "run_metadata.txt") << "original metadata";
    std::filesystem::create_directory_symlink(target, link);

    EXPECT_FALSE(exportTruthTelemetry(*result, link));
    EXPECT_TRUE(std::filesystem::is_symlink(link));
    EXPECT_EQ(readFile(target / "truth_trajectory.csv"), "original CSV");
    EXPECT_EQ(readFile(target / "run_metadata.txt"), "original metadata");
}

TEST(TruthTelemetryTest, RejectsSharedParentWithoutStickyProtection) {
    const auto result = stationaryResult();
    ASSERT_TRUE(result);
    const test::TemporaryDirectory temporary;
    const auto parent = temporary.path() / "shared";
    ASSERT_TRUE(std::filesystem::create_directory(parent));
    std::filesystem::permissions(parent, std::filesystem::perms::owner_all |
                                          std::filesystem::perms::group_write);
    EXPECT_FALSE(exportTruthTelemetry(*result, parent / "output"));
    EXPECT_TRUE(std::filesystem::is_empty(parent));
}

struct Replacement {
    std::filesystem::path output;
    std::filesystem::path moved;
    std::filesystem::path target;
    detail::ExportCheckpoint checkpoint;
    bool fail;
    bool called = false;
};

bool replaceDestination(detail::ExportCheckpoint checkpoint, void* context) {
    auto& replacement = *static_cast<Replacement*>(context);
    if (checkpoint != replacement.checkpoint) {
        return true;
    }
    replacement.called = true;
    if (checkpoint == detail::ExportCheckpoint::CsvHeaderWritten) {
        EXPECT_FALSE(readFile(replacement.output / "truth_trajectory.csv").empty());
    }
    std::filesystem::rename(replacement.output, replacement.moved);
    std::filesystem::create_directory_symlink(replacement.target, replacement.output);
    return !replacement.fail;
}

void verifyReplacement(detail::ExportCheckpoint checkpoint, bool fail) {
    const auto result = stationaryResult();
    ASSERT_TRUE(result);
    const test::TemporaryDirectory temporary;
    Replacement replacement{temporary.path() / "output", temporary.path() / "moved",
                            temporary.path() / "unrelated", checkpoint, fail};
    ASSERT_TRUE(std::filesystem::create_directory(replacement.target));
    std::ofstream(replacement.target / "truth_trajectory.csv") << "original CSV";
    std::ofstream(replacement.target / "run_metadata.txt") << "original metadata";

    EXPECT_FALSE(detail::exportTruthTelemetryImpl(*result, replacement.output,
                                                  replaceDestination, &replacement));
    EXPECT_TRUE(replacement.called);
    EXPECT_TRUE(std::filesystem::is_symlink(replacement.output));
    EXPECT_EQ(std::filesystem::read_symlink(replacement.output), replacement.target);
    EXPECT_EQ(readFile(replacement.target / "truth_trajectory.csv"), "original CSV");
    EXPECT_EQ(readFile(replacement.target / "run_metadata.txt"), "original metadata");
    EXPECT_TRUE(std::filesystem::is_empty(replacement.moved));
}

TEST(TruthTelemetryTest, RejectsDestinationReplacedAfterAcquisition) {
    verifyReplacement(detail::ExportCheckpoint::DirectoryAcquired, false);
}

TEST(TruthTelemetryTest, RejectsParentReplacedAfterAcquisition) {
    const auto result = stationaryResult();
    ASSERT_TRUE(result);
    const test::TemporaryDirectory temporary;
    struct ParentReplacement {
        std::filesystem::path parent;
        std::filesystem::path moved;
        bool called = false;
    } replacement{temporary.path() / "parent", temporary.path() / "moved"};
    ASSERT_TRUE(std::filesystem::create_directory(replacement.parent));
    std::ofstream(replacement.parent / "unrelated") << "original parent sentinel";
    const auto output = replacement.parent / "output";

    const auto replace_parent = [](detail::ExportCheckpoint checkpoint, void* context) {
        auto& paths = *static_cast<ParentReplacement*>(context);
        if (checkpoint == detail::ExportCheckpoint::DirectoryAcquired) {
            paths.called = true;
            std::filesystem::rename(paths.parent, paths.moved);
            const auto replacement_output = paths.parent / "output";
            std::filesystem::create_directories(replacement_output);
            std::ofstream(replacement_output / "truth_trajectory.csv") << "replacement CSV";
            std::ofstream(replacement_output / "run_metadata.txt") << "replacement metadata";
            std::ofstream(paths.parent / "unrelated") << "replacement parent sentinel";
        }
        return true;
    };

    EXPECT_FALSE(detail::exportTruthTelemetryImpl(*result, output, replace_parent, &replacement));
    EXPECT_TRUE(replacement.called);
    EXPECT_EQ(readFile(output / "truth_trajectory.csv"), "replacement CSV");
    EXPECT_EQ(readFile(output / "run_metadata.txt"), "replacement metadata");
    EXPECT_EQ(readFile(replacement.parent / "unrelated"), "replacement parent sentinel");
    EXPECT_EQ(readFile(replacement.moved / "unrelated"), "original parent sentinel");
    EXPECT_TRUE(std::filesystem::is_empty(replacement.moved / "output"));
}

TEST(TruthTelemetryTest, CleansOwnedFilesAfterFailureAndDestinationReplacement) {
    verifyReplacement(detail::ExportCheckpoint::CsvHeaderWritten, true);
}

TEST(TruthTelemetryTest, PartialOutputFailureCleansOnlyOwnedFile) {
    const auto result = stationaryResult();
    ASSERT_TRUE(result);
    const test::TemporaryDirectory temporary;
    const auto output = temporary.path() / "output";
    bool injected = false;
    const auto fail_after_header = [](detail::ExportCheckpoint checkpoint, void* context) {
        if (checkpoint == detail::ExportCheckpoint::CsvHeaderWritten) {
            *static_cast<bool*>(context) = true;
            return false;
        }
        return true;
    };

    const auto sentinel = temporary.path() / "sentinel";
    std::ofstream(sentinel) << "unrelated file";
    EXPECT_FALSE(detail::exportTruthTelemetryImpl(*result, output, fail_after_header, &injected));
    EXPECT_TRUE(injected);
    EXPECT_TRUE(std::filesystem::is_empty(output));
    EXPECT_EQ(readFile(sentinel), "unrelated file");
}

TEST(TruthTelemetryTest, FileCollisionDoesNotFollowSymlinkOrRemoveExistingEntry) {
    const auto result = stationaryResult();
    ASSERT_TRUE(result);
    const test::TemporaryDirectory temporary;
    const auto output = temporary.path() / "output";
    const auto sentinel = temporary.path() / "sentinel";
    std::ofstream(sentinel) << "unrelated file";
    const auto metadata = output / "run_metadata.txt";
    const auto plant_symlink = [](detail::ExportCheckpoint checkpoint, void* context) {
        if (checkpoint == detail::ExportCheckpoint::CsvHeaderWritten) {
            const auto& path = *static_cast<const std::filesystem::path*>(context);
            std::filesystem::create_symlink("../sentinel", path);
        }
        return true;
    };

    auto link = metadata;
    EXPECT_FALSE(detail::exportTruthTelemetryImpl(*result, output, plant_symlink, &link));
    EXPECT_TRUE(std::filesystem::is_symlink(metadata));
    EXPECT_EQ(readFile(sentinel), "unrelated file");
    EXPECT_FALSE(std::filesystem::exists(output / "truth_trajectory.csv"));
}

} // namespace
} // namespace aerotwin
