#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace aerotwin {

using SimulationNanoseconds = std::chrono::duration<std::int64_t, std::nano>;

struct SimulationTimestamp {
    std::uint64_t epoch{};
    SimulationNanoseconds elapsed{};

    bool operator==(const SimulationTimestamp&) const = default;
};

class SimulationClock {
public:
    // Starts unpaused at epoch zero and zero elapsed time.
    [[nodiscard]] static std::optional<SimulationClock>
    create(SimulationNanoseconds timestep) noexcept;

    // Require callers to convert other duration types explicitly.
    template <typename Rep, typename Period>
    static std::optional<SimulationClock> create(std::chrono::duration<Rep, Period>) = delete;
    [[nodiscard]] SimulationTimestamp now() const noexcept { return timestamp_; }
    [[nodiscard]] SimulationNanoseconds timestep() const noexcept { return timestep_; }
    [[nodiscard]] bool isPaused() const noexcept { return paused_; }

    // Returns false while paused or if the next timestamp would overflow.
    [[nodiscard]] bool step() noexcept;
    void pause() noexcept { paused_ = true; }
    void resume() noexcept { paused_ = false; }

    // Starts the next epoch unpaused. Returns false at the maximum epoch.
    // Failed steps and resets leave all clock state unchanged.
    [[nodiscard]] bool reset() noexcept;

private:
    // Boundary fixture can exercise epoch exhaustion without a public state setter.
    friend class SimulationClockBoundaryTest;

    explicit SimulationClock(SimulationNanoseconds timestep) noexcept : timestep_(timestep) {}

    SimulationNanoseconds timestep_;
    SimulationTimestamp timestamp_{};
    bool paused_{false};
};

} // namespace aerotwin
