#include <aerotwin/sensors/SensorSamplingPipeline.hpp>

#include <aerotwin/simulation/VehicleState.hpp>

namespace aerotwin {
namespace {

enum class Due { No, Now, Missed };

Due dueAt(SimulationNanoseconds period, std::optional<SimulationNanoseconds> last,
          SimulationNanoseconds elapsed) {
    // Disabled channels have period zero. Before the first call, zero is due.
    if (period == SimulationNanoseconds::zero()) {
        return Due::No;
    }
    if (!last) {
        return elapsed == SimulationNanoseconds::zero() ? Due::Now : Due::Missed;
    }
    // Compare distances instead of forming a potentially unrepresentable next timestamp.
    const auto until_due = period - (*last % period);
    const auto advance = elapsed - *last;
    if (advance > until_due) {
        return Due::Missed;
    }
    return advance == until_due ? Due::Now : Due::No;
}

} // namespace

std::optional<SensorSamplingPipeline> SensorSamplingPipeline::create(
    const std::optional<GpsSamplingConfig>& gps, const std::optional<ImuSamplingConfig>& imu) {
    if ((!gps && !imu) || (gps && gps->period <= SimulationNanoseconds::zero()) ||
        (imu && imu->period <= SimulationNanoseconds::zero())) {
        return std::nullopt;
    }
    return SensorSamplingPipeline(gps, imu);
}

SensorSamplingPipeline::SensorSamplingPipeline(const std::optional<GpsSamplingConfig>& gps,
                                               const std::optional<ImuSamplingConfig>& imu) {
    if (gps) {
        gps_.emplace(gps->sensor);
        gps_period_ = gps->period;
    }
    if (imu) {
        imu_.emplace(imu->sensor);
        imu_period_ = imu->period;
    }
}

void SensorSamplingPipeline::reset(std::uint64_t epoch) noexcept {
    epoch_ = epoch;
    last_elapsed_.reset();
    if (gps_) {
        gps_->reset();
    }
    if (imu_) {
        imu_->reset();
    }
}

SamplingResult SensorSamplingPipeline::sample(const VehicleState& truth,
                                               SimulationTimestamp timestamp,
                                               const ImuKinematics* kinematics) {
    if (truth.timestamp() != timestamp) {
        return SamplingError::TimestampMismatch;
    }
    if (timestamp.epoch != epoch_) {
        return SamplingError::EpochMismatch;
    }
    if (last_elapsed_) {
        if (timestamp.elapsed < *last_elapsed_) {
            return SamplingError::NonmonotonicTimestamp;
        }
        if (timestamp.elapsed == *last_elapsed_) {
            return SamplingError::DuplicateTimestamp;
        }
    }

    const auto gps_due = dueAt(gps_period_, last_elapsed_, timestamp.elapsed);
    const auto imu_due = dueAt(imu_period_, last_elapsed_, timestamp.elapsed);
    if (gps_due == Due::Missed || imu_due == Due::Missed) {
        return SamplingError::MissedSample;
    }
    if (imu_due == Due::Now) {
        if (!kinematics) {
            return SamplingError::MissingImuKinematics;
        }
        if (kinematics->timestamp() != timestamp) {
            return SamplingError::ImuKinematicsTimestampMismatch;
        }
    }

    // Sample copies so an error in either channel cannot consume the owned streams.
    MeasurementBatch batch{};
    std::optional<GpsSensor> gps_trial;
    std::optional<ImuSensor> imu_trial;
    if (gps_due == Due::Now) {
        auto& sensor = gps_trial.emplace(*gps_);
        const auto measurement = sensor.measure(truth, timestamp);
        if (!measurement) {
            return SamplingError::GpsMeasurementFailed;
        }
        batch[0].emplace(*measurement);
    }
    if (imu_due == Due::Now) {
        auto& sensor = imu_trial.emplace(*imu_);
        const auto measurement = sensor.measure(truth, *kinematics, timestamp);
        if (!measurement) {
            return SamplingError::ImuMeasurementFailed;
        }
        batch[gps_trial ? 1 : 0].emplace(*measurement);
    }

    if (gps_trial) {
        *gps_ = *gps_trial;
    }
    if (imu_trial) {
        *imu_ = *imu_trial;
    }
    last_elapsed_ = timestamp.elapsed;
    return batch;
}

} // namespace aerotwin
