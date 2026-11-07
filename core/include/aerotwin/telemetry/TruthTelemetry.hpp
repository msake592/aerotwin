#pragma once

#include <aerotwin/simulation/ScenarioRunner.hpp>

#include <filesystem>

namespace aerotwin {

// Creates a new output directory under an existing parent. Existing paths are
// rejected. On failure, owned files are cleaned up where safe. The directory may remain.
[[nodiscard]] bool exportTruthTelemetry(const ScenarioResult& result,
                                        const std::filesystem::path& output_directory);

} // namespace aerotwin
