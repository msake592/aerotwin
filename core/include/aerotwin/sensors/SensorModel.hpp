#pragma once

#include <aerotwin/sensors/Measurements.hpp>

namespace aerotwin {

class VehicleState;

// Generation-side interface. Consumers include Measurements.hpp instead.
template <typename Measurement>
class SensorModel {
public:
    virtual ~SensorModel() = default;

    // The view is borrowed from the model.
    [[nodiscard]] virtual std::string_view sensorId() const noexcept = 0;

    // Truth is borrowed for this call and must describe the supplied acquisition
    // time and epoch. Implementations reject a mismatch and return an owned value
    // tagged with this sensor's ID and the supplied timestamp on success.
    [[nodiscard]] virtual std::optional<Measurement> measure(
        const VehicleState& truth, SimulationTimestamp acquisition_timestamp) = 0;
};

} // namespace aerotwin
