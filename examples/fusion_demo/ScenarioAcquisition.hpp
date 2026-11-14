#pragma once

#include <aerotwin/sensors/SensorSamplingPipeline.hpp>
#include <aerotwin/simulation/ScenarioRunner.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

namespace aerotwin::fusion_demo {

struct AcquisitionHistory {
    ScenarioResult scenario;
    // Batch i belongs to scenario.samples()[i]. Slot order is acquisition order.
    std::vector<MeasurementBatch> batches;
    std::size_t gps_count;
    std::size_t imu_count;
    SimulationNanoseconds gps_period;
    std::uint64_t gps_seed;
    ImuNoiseSeeds imu_seeds;
    Vector3 gravity_world_mps2;
};

enum class AcquisitionError {
    ScenarioConstruction,
    ScenarioExecution,
    SensorConfiguration,
    ImuKinematics,
    Sampling,
    TimestampMismatch,
    AcquisitionOrder,
};

struct AcquisitionFailure {
    AcquisitionError error;
    std::optional<SimulationTimestamp> timestamp{};
    std::optional<SamplingError> sampling_error{};
};

using AcquisitionResult = std::variant<AcquisitionHistory, AcquisitionFailure>;

[[nodiscard]] AcquisitionResult acquireScenario(
    SimulationNanoseconds gps_period = SimulationNanoseconds{1'000'000'000},
    ImuNoiseSeeds imu_seeds = {43, 44});

} // namespace aerotwin::fusion_demo
