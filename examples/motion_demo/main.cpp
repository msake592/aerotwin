#include <aerotwin/telemetry/TruthTelemetry.hpp>

#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>

namespace {

void printPose(const char* label, const aerotwin::VehicleState& state) {
    const auto p = state.positionWorldM();
    const auto q = state.orientationWorldFromBody();
    std::cout << label << " position ENU (m): (" << p.x() << ", " << p.y() << ", " << p.z()
              << ")\n"
              << label << " orientation body FLU to world ENU (WXYZ): (" << q.w() << ", "
              << q.x() << ", " << q.y() << ", " << q.z() << ")\n";
}

} // namespace

int main(int argc, char* argv[]) {
    using namespace aerotwin;

    if (argc > 2) {
        std::cerr << "Usage: aerotwin_motion_demo [new-output-directory]\n";
        return 1;
    }
    std::filesystem::path output;
    if (argc == 2) {
        output = argv[1];
    } else {
        std::error_code error;
        const auto temporary = std::filesystem::temp_directory_path(error);
        if (error) {
            std::cerr << "Could not find the system temporary directory: " << error.message() << '\n';
            return 1;
        }
        output = temporary / "aerotwin_m1_motion_demo";
    }

    const auto initial = VehicleState::create(
        Vector3(10.0, -5.0, 2.0), Quaternion::Identity(), Vector3(2.0, 1.0, 0.5),
        Vector3(0.0, 0.0, std::numbers::pi / 4.0), {});
    if (!initial) {
        std::cerr << "Could not construct the reference initial state\n";
        return 1;
    }
    const auto config = ScenarioConfig::create(
        *initial, SimulationNanoseconds{100'000'000}, 100, 42, "m1_motion_demo");
    if (!config) {
        std::cerr << "Could not construct the reference scenario\n";
        return 1;
    }
    const auto result = runScenario(*config);
    if (!result) {
        std::cerr << "Reference scenario failed to complete\n";
        return 1;
    }
    if (!exportTruthTelemetry(*result, output)) {
        std::cerr << "Could not export truth telemetry to " << output
                  << ". Use a new directory under an existing writable parent.\n";
        return 1;
    }

    const auto& first = result->samples().front();
    const auto& last = result->samples().back();
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10)
              << "Scenario: " << config->identifier().value_or("") << '\n'
              << "Steps: " << config->stepCount() << '\n'
              << "Samples: " << result->samples().size() << '\n'
              << "Timestep: " << config->timestep().count() << " ns\n"
              << "Simulated duration: " << last.timestamp().elapsed.count() << " ns\n";
    printPose("Initial", first);
    printPose("Final", last);
    std::cout << "CSV: " << output / "truth_trajectory.csv" << '\n'
              << "Metadata: " << output / "run_metadata.txt" << '\n';
    return 0;
}
