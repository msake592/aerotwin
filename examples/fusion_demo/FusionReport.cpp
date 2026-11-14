#include "FusionReport.hpp"

#include <array>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <type_traits>
#include <utility>

namespace aerotwin::fusion_demo {
namespace detail {

void writeCsvField(std::ostream& stream, std::string_view field) {
    if (field.find_first_of(",\"\r\n") == std::string_view::npos) {
        stream << field;
        return;
    }
    stream << '"';
    for (const char character : field) {
        if (character == '"') {
            stream << '"';
        }
        stream << character;
    }
    stream << '"';
}

} // namespace detail

namespace {

class CsvRow {
public:
    explicit CsvRow(std::ostream& stream) : stream_(stream) {}

    template <typename... Fields>
    void write(const Fields&... fields) {
        (writeField(fields), ...);
    }

    void skip(std::size_t count) {
        for (std::size_t i = 0; i < count; ++i) {
            writeField("");
        }
    }

    void finish() { stream_ << '\n'; }

private:
    template <typename Field>
    void writeField(const Field& field) {
        if (!first_) {
            stream_ << ',';
        }
        first_ = false;
        if constexpr (std::is_convertible_v<const Field&, std::string_view>) {
            detail::writeCsvField(stream_, field);
        } else {
            stream_ << field;
        }
    }

    std::ostream& stream_;
    bool first_ = true;
};

constexpr unsigned kSchemaVersion = 1;
constexpr std::array kEnu{"e", "n", "u"};
constexpr std::array kFlu{"f", "l", "u"};
constexpr std::array kState{"pe", "pn", "pu", "ve", "vn", "vu"};
constexpr std::size_t kConfigFields = 10 + 3 * kEnu.size();
constexpr std::size_t upperTriangle(std::size_t dimension) {
    return dimension * (dimension + 1) / 2;
}
constexpr std::size_t kMeasurementFields = kEnu.size() + upperTriangle(kEnu.size());
constexpr std::size_t kEstimatorFields = kState.size() + upperTriangle(kState.size());
constexpr std::size_t kGpsFields = kMeasurementFields + 2 * kEstimatorFields + 8;
constexpr std::size_t kImuFields = 2 * kMeasurementFields;
constexpr std::size_t kSummaryFields = 11 + 2 * (kEnu.size() + 1);

template <std::size_t N>
void axesHeader(CsvRow& row, std::string_view prefix, const std::array<const char*, N>& axes,
                std::string_view unit) {
    for (const auto* axis : axes) {
        row.write(std::string(prefix) + axis + std::string(unit));
    }
}

template <std::size_t N>
void covarianceHeader(CsvRow& row, std::string_view prefix, const std::array<const char*, N>& axes,
                      std::string_view unit = {}) {
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = i; j < N; ++j) {
            row.write(std::string(prefix) + axes[i] + "_" + axes[j] + std::string(unit));
        }
    }
}

void vectorFields(CsvRow& row, const Vector3& vector) {
    row.write(vector.x(), vector.y(), vector.z());
}

template <typename Matrix>
void covarianceFields(CsvRow& row, const Matrix& matrix) {
    for (Eigen::Index i = 0; i < matrix.rows(); ++i) {
        for (Eigen::Index j = i; j < matrix.cols(); ++j) {
            row.write(matrix(i, j));
        }
    }
}

void writeHeader(std::ostream& out) {
    CsvRow row(out);
    row.write("record_type", "schema_version", "epoch", "elapsed_ns", "sensor_id");
    row.write("scenario_id", "scenario_seed", "step_count", "timestep_ns", "gps_period_ns",
              "imu_period_ns", "gps_seed", "accelerometer_seed", "gyroscope_seed");
    axesHeader(row, "gravity_enu_", kEnu, "_mps2");
    row.write("acceleration_spectral_density_m2ps3");
    axesHeader(row, "initial_truth_position_enu_", kEnu, "_m");
    axesHeader(row, "initial_truth_velocity_enu_", kEnu, "_mps");
    axesHeader(row, "gps_position_enu_", kEnu, "_m");
    covarianceHeader(row, "gps_cov_", kEnu, "_m2");
    for (const std::string phase : {"prior_", "posterior_"}) {
        axesHeader(row, phase + "position_enu_", kEnu, "_m");
        axesHeader(row, phase + "velocity_enu_", kEnu, "_mps");
        covarianceHeader(row, phase + "cov_", kState);
    }
    row.write("gps_nis", "gps_nis_dof", "full_nees", "full_nees_dof", "position_nees",
              "position_nees_dof", "velocity_nees", "velocity_nees_dof");
    axesHeader(row, "imu_specific_force_flu_", kFlu, "_mps2");
    covarianceHeader(row, "imu_specific_force_cov_", kFlu, "_m2ps4");
    axesHeader(row, "imu_angular_velocity_flu_", kFlu, "_radps");
    covarianceHeader(row, "imu_angular_velocity_cov_", kFlu, "_rad2ps2");
    row.write("truth_count", "gps_count", "imu_count", "prediction_count", "gps_update_count",
              "rejected_observation_count", "gps_nis_count", "full_nees_count", "position_nees_count",
              "velocity_nees_count", "rmse_sample_count");
    axesHeader(row, "position_rmse_enu_", kEnu, "_m");
    row.write("position_rmse_3d_m");
    axesHeader(row, "velocity_rmse_enu_", kEnu, "_mps");
    row.write("velocity_rmse_3d_mps");
    row.finish();
}

void writeRecord(CsvRow& row, std::string_view record_type, SimulationTimestamp timestamp,
                 std::string_view sensor_id = {}) {
    row.write(record_type, kSchemaVersion, timestamp.epoch, timestamp.elapsed.count(), sensor_id);
}

void writeConfiguration(std::ostream& out, const AcquisitionHistory& history) {
    CsvRow row(out);
    const auto& config = history.scenario.config();
    const auto& initial = config.initialState();
    writeRecord(row, "configuration", initial.timestamp());
    row.write(config.identifier().value_or(""), config.seed(), config.stepCount(),
              config.timestep().count(), history.gps_period.count(), config.timestep().count(),
              history.gps_seed, history.imu_seeds.accelerometer, history.imu_seeds.gyroscope);
    vectorFields(row, history.gravity_world_mps2);
    row.write(kAccelerationSpectralDensityM2ps3);
    vectorFields(row, initial.positionWorldM());
    vectorFields(row, initial.linearVelocityWorldMps());
    row.skip(kGpsFields + kImuFields + kSummaryFields);
    row.finish();
}

void writeGps(std::ostream& out, const GpsEvaluation& event, const GpsMeasurement& measurement) {
    CsvRow row(out);
    writeRecord(row, "gps_evaluation", measurement.acquisition().timestamp(),
                measurement.acquisition().sensorId());
    row.skip(kConfigFields);
    vectorFields(row, measurement.positionWorldM());
    covarianceFields(row, measurement.positionCovarianceWorldM2());
    for (const auto* state : {&event.prior, &event.posterior_truth.estimate}) {
        vectorFields(row, state->positionWorldM());
        vectorFields(row, state->velocityWorldMps());
        covarianceFields(row, state->covariance());
    }
    for (const auto score : {event.nis, event.full_nees, event.position_nees, event.velocity_nees}) {
        row.write(score.value, score.degrees_of_freedom);
    }
    row.skip(kImuFields + kSummaryFields);
    row.finish();
}

void writeImu(std::ostream& out, const ImuMeasurement& measurement) {
    CsvRow row(out);
    writeRecord(row, "imu_diagnostic", measurement.acquisition().timestamp(),
                measurement.acquisition().sensorId());
    row.skip(kConfigFields + kGpsFields);
    vectorFields(row, measurement.specificForceBodyMps2());
    covarianceFields(row, measurement.specificForceCovarianceBodyM2ps4());
    vectorFields(row, measurement.angularVelocityBodyRadps());
    covarianceFields(row, measurement.angularVelocityCovarianceBodyRad2ps2());
    row.skip(kSummaryFields);
    row.finish();
}

void writeSummary(std::ostream& out, const FusionEvaluation& run) {
    CsvRow row(out);
    const auto& ticks = run.acquisition.scenario.samples();
    const auto score_count = run.gps_events.size();
    writeRecord(row, "summary", ticks.back().timestamp());
    row.skip(kConfigFields + kGpsFields + kImuFields);
    row.write(ticks.size(), run.acquisition.gps_count, run.acquisition.imu_count,
              run.prediction_count, run.gps_update_count,
              run.acquisition.gps_count - run.gps_update_count, score_count, score_count,
              score_count, score_count, run.rmse.sample_count);
    vectorFields(row, run.rmse.position_axis_m);
    row.write(run.rmse.position_3d_m);
    vectorFields(row, run.rmse.velocity_axis_mps);
    row.write(run.rmse.velocity_3d_mps);
    row.finish();
}

std::string_view nameOf(AcquisitionError value) {
    switch (value) {
    case AcquisitionError::ScenarioConstruction:
        return "AcquisitionError::ScenarioConstruction";
    case AcquisitionError::ScenarioExecution:
        return "AcquisitionError::ScenarioExecution";
    case AcquisitionError::SensorConfiguration:
        return "AcquisitionError::SensorConfiguration";
    case AcquisitionError::ImuKinematics:
        return "AcquisitionError::ImuKinematics";
    case AcquisitionError::Sampling:
        return "AcquisitionError::Sampling";
    case AcquisitionError::TimestampMismatch:
        return "AcquisitionError::TimestampMismatch";
    case AcquisitionError::AcquisitionOrder:
        return "AcquisitionError::AcquisitionOrder";
    }
    return "UnknownAcquisitionError";
}

std::string_view nameOf(SamplingError value) {
    switch (value) {
    case SamplingError::TimestampMismatch:
        return "SamplingError::TimestampMismatch";
    case SamplingError::EpochMismatch:
        return "SamplingError::EpochMismatch";
    case SamplingError::DuplicateTimestamp:
        return "SamplingError::DuplicateTimestamp";
    case SamplingError::NonmonotonicTimestamp:
        return "SamplingError::NonmonotonicTimestamp";
    case SamplingError::MissedSample:
        return "SamplingError::MissedSample";
    case SamplingError::MissingImuKinematics:
        return "SamplingError::MissingImuKinematics";
    case SamplingError::ImuKinematicsTimestampMismatch:
        return "SamplingError::ImuKinematicsTimestampMismatch";
    case SamplingError::GpsMeasurementFailed:
        return "SamplingError::GpsMeasurementFailed";
    case SamplingError::ImuMeasurementFailed:
        return "SamplingError::ImuMeasurementFailed";
    }
    return "UnknownSamplingError";
}

std::string_view nameOf(FusionStage value) {
    switch (value) {
    case FusionStage::AcquisitionHistory:
        return "AcquisitionHistory";
    case FusionStage::AcquisitionAlignment:
        return "AcquisitionAlignment";
    case FusionStage::Initialization:
        return "Initialization";
    case FusionStage::Prediction:
        return "Prediction";
    case FusionStage::GpsNis:
        return "GpsNis";
    case FusionStage::GpsCorrection:
        return "GpsCorrection";
    case FusionStage::Nees:
        return "Nees";
    case FusionStage::Rmse:
        return "Rmse";
    }
    return "UnknownFusionStage";
}

std::string_view nameOf(EstimatorStateError value) {
    switch (value) {
    case EstimatorStateError::InvalidState:
        return "EstimatorStateError::InvalidState";
    case EstimatorStateError::InvalidTimestamp:
        return "EstimatorStateError::InvalidTimestamp";
    case EstimatorStateError::InvalidCovariance:
        return "EstimatorStateError::InvalidCovariance";
    }
    return "UnknownEstimatorStateError";
}

std::string_view nameOf(PredictionError value) {
    switch (value) {
    case PredictionError::InvalidProcessNoise:
        return "PredictionError::InvalidProcessNoise";
    case PredictionError::InvalidTargetTimestamp:
        return "PredictionError::InvalidTargetTimestamp";
    case PredictionError::EpochMismatch:
        return "PredictionError::EpochMismatch";
    case PredictionError::NonForwardTime:
        return "PredictionError::NonForwardTime";
    case PredictionError::NumericalOverflow:
        return "PredictionError::NumericalOverflow";
    case PredictionError::InvalidPropagatedCovariance:
        return "PredictionError::InvalidPropagatedCovariance";
    }
    return "UnknownPredictionError";
}

std::string_view nameOf(GpsUpdateError value) {
    switch (value) {
    case GpsUpdateError::EpochMismatch:
        return "GpsUpdateError::EpochMismatch";
    case GpsUpdateError::TimestampMismatch:
        return "GpsUpdateError::TimestampMismatch";
    case GpsUpdateError::InvalidInnovationCovariance:
        return "GpsUpdateError::InvalidInnovationCovariance";
    case GpsUpdateError::UnsolvableInnovationCovariance:
        return "GpsUpdateError::UnsolvableInnovationCovariance";
    case GpsUpdateError::NumericalFailure:
        return "GpsUpdateError::NumericalFailure";
    case GpsUpdateError::InvalidPosteriorCovariance:
        return "GpsUpdateError::InvalidPosteriorCovariance";
    }
    return "UnknownGpsUpdateError";
}

std::string_view nameOf(EvaluationError value) {
    switch (value) {
    case EvaluationError::EmptySamples:
        return "EvaluationError::EmptySamples";
    case EvaluationError::EpochMismatch:
        return "EvaluationError::EpochMismatch";
    case EvaluationError::TimestampMismatch:
        return "EvaluationError::TimestampMismatch";
    case EvaluationError::NumericalFailure:
        return "EvaluationError::NumericalFailure";
    case EvaluationError::InvalidCovariance:
        return "EvaluationError::InvalidCovariance";
    case EvaluationError::UnsolvableCovariance:
        return "EvaluationError::UnsolvableCovariance";
    }
    return "UnknownEvaluationError";
}

void writeTimestamp(std::ostream& errors, SimulationTimestamp timestamp) {
    errors << " epoch=" << timestamp.epoch << " elapsed_ns=" << timestamp.elapsed.count();
}

void writeFailure(std::ostream& errors, const AcquisitionFailure& failure) {
    errors << "Fusion demo failed: stage=Acquisition error=" << nameOf(failure.error);
    if (failure.sampling_error) {
        errors << " cause=" << nameOf(*failure.sampling_error);
    }
    if (failure.timestamp) {
        writeTimestamp(errors, *failure.timestamp);
    }
    errors << '\n';
}

void writeFailure(std::ostream& errors, const FusionFailure& failure) {
    errors << "Fusion demo failed: stage=" << nameOf(failure.stage);
    std::visit([&errors](const auto& error) {
        if constexpr (!std::is_same_v<std::decay_t<decltype(error)>, std::monostate>) {
            errors << " error=" << nameOf(error);
        }
    }, failure.error);
    writeTimestamp(errors, failure.timestamp);
    if (failure.observation) {
        errors << " sensor_id=" << failure.observation->sensorId();
    }
    if (failure.sample_index) {
        errors << " sample_index=" << *failure.sample_index;
    }
    errors << '\n';
}

} // namespace

std::string serializeFusionCsv(const FusionEvaluation& run) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    writeHeader(out);
    writeConfiguration(out, run.acquisition);
    const auto& batches = run.acquisition.batches;
    std::size_t next_gps = 0;
    for (const auto& batch : batches) {
        for (const auto& slot : batch) {
            if (!slot) {
                continue;
            }
            if (const auto* gps = std::get_if<GpsMeasurement>(&*slot)) {
                writeGps(out, run.gps_events[next_gps++], *gps);
            } else {
                writeImu(out, std::get<ImuMeasurement>(*slot));
            }
        }
    }
    writeSummary(out, run);
    return std::move(out).str();
}

int writeFusionReport(AcquisitionResult acquisition, std::ostream& output, std::ostream& errors) {
    if (const auto* failure = std::get_if<AcquisitionFailure>(&acquisition)) {
        writeFailure(errors, *failure);
        return 1;
    }
    const auto evaluation = evaluateFusion(std::get<AcquisitionHistory>(std::move(acquisition)));
    if (const auto* failure = std::get_if<FusionFailure>(&evaluation)) {
        writeFailure(errors, *failure);
        return 1;
    }

    const auto report = serializeFusionCsv(std::get<FusionEvaluation>(evaluation));
    output.write(report.data(), static_cast<std::streamsize>(report.size()));
    output.flush();
    if (!output) {
        errors << "Fusion demo failed: could not write CSV to stdout\n";
        return 1;
    }
    return 0;
}

} // namespace aerotwin::fusion_demo
