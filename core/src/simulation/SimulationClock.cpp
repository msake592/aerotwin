#include <aerotwin/simulation/SimulationClock.hpp>

#include <limits>

namespace aerotwin {

std::optional<SimulationClock> SimulationClock::create(SimulationNanoseconds timestep) noexcept {
    if (timestep <= SimulationNanoseconds::zero()) {
        return std::nullopt;
    }
    return SimulationClock(timestep);
}

bool SimulationClock::step() noexcept {
    if (paused_ || timestamp_.elapsed > SimulationNanoseconds::max() - timestep_) {
        return false;
    }
    timestamp_.elapsed += timestep_;
    return true;
}

bool SimulationClock::reset() noexcept {
    if (timestamp_.epoch == std::numeric_limits<std::uint64_t>::max()) {
        return false;
    }
    timestamp_ = {timestamp_.epoch + 1, SimulationNanoseconds::zero()};
    resume();
    return true;
}

} // namespace aerotwin
