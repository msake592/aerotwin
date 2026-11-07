#pragma once

#include <aerotwin/telemetry/TruthTelemetry.hpp>

namespace aerotwin::detail {

enum class ExportCheckpoint { DirectoryAcquired, CsvHeaderWritten };

// Internal checkpoints let tests replace the directory or inject a failure
// synchronously. The public entry point supplies no hook.
using ExportHook = bool (*)(ExportCheckpoint, void*);

bool exportTruthTelemetryImpl(const ScenarioResult& result,
                              const std::filesystem::path& output_directory,
                              ExportHook hook, void* context);

} // namespace aerotwin::detail
