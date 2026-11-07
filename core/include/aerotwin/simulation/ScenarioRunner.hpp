#pragma once

#include <aerotwin/simulation/VehicleState.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace aerotwin {

class ScenarioConfig {
public:
    static constexpr std::int64_t kMaxSteps = 100'000;

    // The initial timestamp must match a fresh clock. The identifier, when
    // present, is 1–64 ASCII letters, digits, underscores, hyphens or periods.
    [[nodiscard]] static std::optional<ScenarioConfig> create(
        const VehicleState& initial_state, SimulationNanoseconds timestep,
        std::int64_t step_count, std::uint64_t seed,
        std::optional<std::string_view> identifier = std::nullopt);

    // Match SimulationClock's explicit nanosecond conversion policy.
    template <typename Rep, typename Period>
    static std::optional<ScenarioConfig> create(
        const VehicleState&, std::chrono::duration<Rep, Period>, std::int64_t, std::uint64_t,
        std::optional<std::string_view> = std::nullopt) = delete;

    [[nodiscard]] const VehicleState& initialState() const noexcept { return initial_state_; }
    [[nodiscard]] SimulationNanoseconds timestep() const noexcept { return timestep_; }
    [[nodiscard]] std::int64_t stepCount() const noexcept { return step_count_; }
    [[nodiscard]] std::uint64_t seed() const noexcept { return seed_; }
    [[nodiscard]] std::optional<std::string_view> identifier() const noexcept;

private:
    ScenarioConfig(const VehicleState& initial_state, SimulationNanoseconds timestep,
                   std::int64_t step_count, std::uint64_t seed, std::string identifier);

    VehicleState initial_state_;
    SimulationNanoseconds timestep_;
    std::int64_t step_count_;
    std::uint64_t seed_;
    std::string identifier_;
};

class ScenarioResult {
public:
    static constexpr unsigned kSchemaVersion = 1;

    ScenarioResult(ScenarioResult&&) noexcept = default;
    ScenarioResult& operator=(ScenarioResult&&) noexcept = default;

    [[nodiscard]] const ScenarioConfig& config() const noexcept { return config_; }
    [[nodiscard]] const std::vector<VehicleState>& samples() const noexcept { return samples_; }

private:
    friend std::optional<ScenarioResult> runScenario(const ScenarioConfig& config);

    ScenarioResult(const ScenarioConfig& config, std::vector<VehicleState> samples);

    ScenarioConfig config_;
    std::vector<VehicleState> samples_;
};

// Includes the initial snapshot. Returns nullopt on clock, propagation or allocation failure.
[[nodiscard]] std::optional<ScenarioResult> runScenario(const ScenarioConfig& config);

} // namespace aerotwin
