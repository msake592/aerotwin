#include <aerotwin/simulation/ScenarioRunner.hpp>

#include <aerotwin/simulation/KinematicIntegrator.hpp>

#include <new>
#include <utility>

namespace aerotwin {

std::optional<ScenarioConfig> ScenarioConfig::create(
    const VehicleState& initial_state, SimulationNanoseconds timestep,
    std::int64_t step_count, std::uint64_t seed, std::optional<std::string_view> identifier) {
    const auto clock = SimulationClock::create(timestep);
    if (!clock || initial_state.timestamp() != clock->now() ||
        step_count < 0 || step_count > kMaxSteps ||
        step_count > SimulationNanoseconds::max().count() / timestep.count()) {
        return std::nullopt;
    }
    constexpr std::string_view allowed =
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-";
    if (identifier && (identifier->empty() || identifier->size() > 64 ||
                       identifier->find_first_not_of(allowed) != std::string_view::npos)) {
        return std::nullopt;
    }
    return ScenarioConfig(initial_state, timestep, step_count, seed,
                          std::string(identifier.value_or("")));
}

ScenarioConfig::ScenarioConfig(const VehicleState& initial_state, SimulationNanoseconds timestep,
                               std::int64_t step_count, std::uint64_t seed, std::string identifier)
    : initial_state_(initial_state), timestep_(timestep), step_count_(step_count),
      seed_(seed), identifier_(std::move(identifier)) {}

std::optional<std::string_view> ScenarioConfig::identifier() const noexcept {
    if (identifier_.empty()) {
        return std::nullopt;
    }
    return identifier_;
}

ScenarioResult::ScenarioResult(const ScenarioConfig& config, std::vector<VehicleState> samples)
    : config_(config), samples_(std::move(samples)) {}

std::optional<ScenarioResult> runScenario(const ScenarioConfig& config) {
    auto clock = SimulationClock::create(config.timestep());
    if (!clock || clock->now() != config.initialState().timestamp()) {
        return std::nullopt;
    }

    try {
        std::vector<VehicleState> samples;
        const auto sample_count = static_cast<std::uint64_t>(config.stepCount()) + 1;
        if (sample_count > samples.max_size()) {
            return std::nullopt;
        }
        samples.reserve(static_cast<std::size_t>(sample_count));
        samples.push_back(config.initialState());
        for (std::int64_t step = 0; step < config.stepCount(); ++step) {
            if (!clock->step()) {
                return std::nullopt;
            }
            const auto next = integrateConstantVelocity(samples.back(), clock->now());
            if (!next) {
                return std::nullopt;
            }
            samples.push_back(*next);
        }
        return ScenarioResult(config, std::move(samples));
    } catch (const std::bad_alloc&) {
        return std::nullopt;
    }
}

} // namespace aerotwin
