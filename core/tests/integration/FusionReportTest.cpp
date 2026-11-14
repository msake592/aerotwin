#include "FusionReport.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <locale>
#include <sstream>
#include <utility>

namespace aerotwin::fusion_demo {
namespace {

std::vector<std::string> splitRow(const std::string& line) {
    // The default scenario and sensor IDs are restricted to unquoted ASCII.
    // Quoting of arbitrary text is exercised separately.
    std::vector<std::string> fields;
    std::size_t begin = 0;
    for (auto comma = line.find(','); comma != std::string::npos; comma = line.find(',', begin)) {
        fields.push_back(line.substr(begin, comma - begin));
        begin = comma + 1;
    }
    fields.push_back(line.substr(begin));
    return fields;
}

struct Csv {
    explicit Csv(const std::string& text) {
        std::istringstream input(text);
        std::string line;
        std::getline(input, line);
        header = splitRow(line);
        while (std::getline(input, line)) {
            rows.push_back(splitRow(line));
        }
    }

    const std::string& field(std::size_t row, std::string_view name) const {
        const auto it = std::find(header.begin(), header.end(), name);
        return rows.at(row).at(static_cast<std::size_t>(it - header.begin()));
    }

    double number(std::size_t row, std::string_view name) const {
        std::istringstream input(field(row, name));
        input.imbue(std::locale::classic());
        double value = 0;
        input >> value;
        EXPECT_TRUE(input && input.eof()) << name;
        return value;
    }

    std::vector<std::string> header;
    std::vector<std::vector<std::string>> rows;
};

class CommaDecimal : public std::numpunct<char> {
    char do_decimal_point() const override { return ','; }
};

TEST(FusionReportTest, SerializesFixedWidthOrderedCsvWithExactMetricValues) {
    auto acquisition = acquireScenario();
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(acquisition));
    const auto evaluation = evaluateFusion(std::get<AcquisitionHistory>(std::move(acquisition)));
    ASSERT_TRUE(std::holds_alternative<FusionEvaluation>(evaluation));
    const auto& run = std::get<FusionEvaluation>(evaluation);
    const auto previous_locale = std::locale::global(std::locale(std::locale::classic(), new CommaDecimal));
    const auto serialized = serializeFusionCsv(run);
    std::locale::global(previous_locale);
    const Csv csv(serialized);

    ASSERT_EQ(csv.header.size(), 132u);
    ASSERT_EQ(csv.rows.size(), 408u);
    for (const auto& row : csv.rows) {
        ASSERT_EQ(row.size(), csv.header.size());
        EXPECT_EQ(row[1], "1");
        EXPECT_EQ(row[2], "0");
    }
    EXPECT_EQ(csv.field(0, "record_type"), "configuration");
    EXPECT_EQ(csv.field(0, "scenario_id"), "m2_fusion_demo");
    EXPECT_EQ(csv.field(0, "elapsed_ns"), "0");
    EXPECT_EQ(csv.field(0, "timestep_ns"), "10000000");
    EXPECT_EQ(csv.field(0, "gps_period_ns"), "1000000000");
    EXPECT_EQ(csv.field(0, "imu_period_ns"), "10000000");
    EXPECT_EQ(csv.field(0, "gps_seed"), "42");
    EXPECT_EQ(csv.field(0, "accelerometer_seed"), "43");
    EXPECT_EQ(csv.field(0, "gyroscope_seed"), "44");
    EXPECT_EQ(csv.number(0, "gravity_enu_u_mps2"), -9.80665);

    std::size_t row = 1;
    for (std::size_t tick = 0; tick < 401; ++tick) {
        const auto elapsed = std::to_string(tick * 10'000'000ULL);
        if (tick % 100 == 0) {
            const auto& event = run.gps_events[tick / 100];
            const auto& posterior = event.posterior_truth.estimate;
            const auto& gps = std::get<GpsMeasurement>(*run.acquisition.batches[tick][0]);
            EXPECT_EQ(csv.field(row, "record_type"), "gps_evaluation");
            EXPECT_EQ(csv.field(row, "sensor_id"), "gps");
            EXPECT_EQ(csv.field(row, "elapsed_ns"), elapsed);
            EXPECT_EQ(csv.number(row, "gps_position_enu_e_m"), gps.positionWorldM().x());
            EXPECT_EQ(csv.number(row, "gps_cov_e_e_m2"), gps.positionCovarianceWorldM2()(0, 0));
            EXPECT_EQ(csv.number(row, "prior_position_enu_e_m"), event.prior.positionWorldM().x());
            EXPECT_EQ(csv.number(row, "posterior_velocity_enu_n_mps"), posterior.velocityWorldMps().y());
            EXPECT_EQ(csv.number(row, "prior_cov_pe_ve"), event.prior.covariance()(0, 3));
            EXPECT_EQ(csv.number(row, "posterior_cov_vu_vu"), posterior.covariance()(5, 5));
            EXPECT_EQ(csv.number(row, "gps_nis"), event.nis.value);
            EXPECT_EQ(csv.number(row, "full_nees"), event.full_nees.value);
            EXPECT_EQ(csv.number(row, "position_nees"), event.position_nees.value);
            EXPECT_EQ(csv.number(row, "velocity_nees"), event.velocity_nees.value);
            EXPECT_EQ(csv.field(row, "gps_nis_dof"), "3");
            EXPECT_EQ(csv.field(row, "full_nees_dof"), "6");
            EXPECT_EQ(csv.field(row, "position_nees_dof"), "3");
            EXPECT_EQ(csv.field(row, "velocity_nees_dof"), "3");
            ++row;
        }
        const auto& imu = std::get<ImuMeasurement>(
            *run.acquisition.batches[tick][tick % 100 == 0 ? 1 : 0]);
        EXPECT_EQ(csv.field(row, "record_type"), "imu_diagnostic");
        EXPECT_EQ(csv.field(row, "sensor_id"), "imu");
        EXPECT_EQ(csv.field(row, "elapsed_ns"), elapsed);
        EXPECT_EQ(csv.number(row, "imu_specific_force_flu_u_mps2"), imu.specificForceBodyMps2().z());
        EXPECT_EQ(csv.number(row, "imu_angular_velocity_flu_f_radps"), imu.angularVelocityBodyRadps().x());
        EXPECT_TRUE(csv.field(row++, "gps_nis").empty());
    }
    ASSERT_EQ(row, csv.rows.size() - 1);
    EXPECT_EQ(csv.field(row, "record_type"), "summary");
    EXPECT_EQ(csv.field(row, "elapsed_ns"), "4000000000");
    EXPECT_EQ(csv.field(row, "truth_count"), "401");
    EXPECT_EQ(csv.field(row, "gps_count"), "5");
    EXPECT_EQ(csv.field(row, "imu_count"), "401");
    EXPECT_EQ(csv.field(row, "prediction_count"), "4");
    EXPECT_EQ(csv.field(row, "gps_update_count"), "5");
    EXPECT_EQ(csv.field(row, "rejected_observation_count"), "0");
    for (const auto* name : {"gps_nis_count", "full_nees_count", "position_nees_count",
                             "velocity_nees_count", "rmse_sample_count"}) {
        EXPECT_EQ(csv.field(row, name), "5");
    }
    constexpr std::array axes{"e", "n", "u"};
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        EXPECT_EQ(csv.number(row, std::string("position_rmse_enu_") + axes[axis] + "_m"),
                  run.rmse.position_axis_m[axis]);
        EXPECT_EQ(csv.number(row, std::string("velocity_rmse_enu_") + axes[axis] + "_mps"),
                  run.rmse.velocity_axis_mps[axis]);
    }
    EXPECT_EQ(csv.number(row, "position_rmse_3d_m"), run.rmse.position_3d_m);
    EXPECT_EQ(csv.number(row, "velocity_rmse_3d_mps"), run.rmse.velocity_3d_mps);
}

// Explicit CSV v1 order, independent of the implementation's header generator.
constexpr std::array kStateCovarianceColumns{
    "pe_pe", "pe_pn", "pe_pu", "pe_ve", "pe_vn", "pe_vu",
    "pn_pn", "pn_pu", "pn_ve", "pn_vn", "pn_vu",
    "pu_pu", "pu_ve", "pu_vn", "pu_vu",
    "ve_ve", "ve_vn", "ve_vu", "vn_vn", "vn_vu", "vu_vu",
};

template <int N, std::size_t ColumnCount>
void expectCovariance(const Csv& csv, std::string_view prefix,
                      const std::array<const char*, ColumnCount>& names,
                      const Eigen::Matrix<double, N, N>& expected) {
    ASSERT_EQ(ColumnCount, N * (N + 1) / 2);
    const auto first = std::find(csv.header.begin(), csv.header.end(),
                                 std::string(prefix) + names.front());
    ASSERT_GE(static_cast<std::size_t>(csv.header.end() - first), ColumnCount);
    Eigen::Matrix<double, N, N> reconstructed;
    std::size_t entry = 0;
    for (int i = 0; i < N; ++i) {
        for (int j = i; j < N; ++j, ++entry) {
            const auto column = std::string(prefix) + names[entry];
            ASSERT_EQ(first[entry], column);
            const double value = csv.number(1, column);
            EXPECT_EQ(value, expected(i, j)) << column;
            reconstructed(i, j) = reconstructed(j, i) = value;
        }
    }
    EXPECT_EQ(reconstructed, expected);
}

TEST(FusionReportTest, PreservesCorrelatedCovarianceOrderAndPhase) {
    // Strictly diagonally dominant with distinct correlations and non-binary decimals.
    EstimatorCovariance6 prior;
    prior << 10, 0.13, -0.24, 0.35, -0.46, 0.57,
             0.13, 11, 0.68, -0.79, 0.81, -0.92,
             -0.24, 0.68, 12, 0.14, -0.25, 0.36,
             0.35, -0.79, 0.14, 13, 0.47, -0.58,
             -0.46, 0.81, -0.25, 0.47, 14, 0.69,
             0.57, -0.92, 0.36, -0.58, 0.69, 15;
    Eigen::Matrix3d measurement;
    measurement << 2, 0.17, -0.29, 0.17, 3, 0.41, -0.29, 0.41, 4;

    auto acquisition = acquireScenario();
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(acquisition));
    auto& history = std::get<AcquisitionHistory>(acquisition);
    auto& slot = history.batches.front()[0];
    const auto& gps = std::get<GpsMeasurement>(*slot);
    const auto correlated = GpsMeasurement::create(
        gps.acquisition(), gps.positionWorldM(), measurement);
    ASSERT_TRUE(correlated);
    slot = *correlated;

    const auto evaluation = evaluateFusion(std::move(history), {}, prior);
    ASSERT_TRUE(std::holds_alternative<FusionEvaluation>(evaluation));
    const auto& run = std::get<FusionEvaluation>(evaluation);
    const auto posterior = run.gps_events.front().posterior_truth.estimate.covariance();
    ASSERT_NE(posterior(0, 3), prior(0, 3));

    const Csv csv(serializeFusionCsv(run));
    ASSERT_EQ(csv.header.size(), 132u);
    ASSERT_EQ(csv.field(1, "record_type"), "gps_evaluation");
    ASSERT_EQ(csv.field(1, "elapsed_ns"), "0");
    constexpr std::array gps_columns{"e_e_m2", "e_n_m2", "e_u_m2", "n_n_m2", "n_u_m2", "u_u_m2"};
    expectCovariance(csv, "gps_cov_", gps_columns, measurement);
    expectCovariance(csv, "prior_cov_", kStateCovarianceColumns, prior);
    expectCovariance(csv, "posterior_cov_", kStateCovarianceColumns, posterior);
}

TEST(FusionReportTest, QuotesCommaQuoteAndLineBreaks) {
    std::ostringstream out;
    detail::writeCsvField(out, "plain");
    out << ',';
    detail::writeCsvField(out, "a,\"b\"\r\nc");
    out << ',';
    detail::writeCsvField(out, "");
    EXPECT_EQ(out.str(), "plain,\"a,\"\"b\"\"\r\nc\",");
}

TEST(FusionReportTest, ReportsTypedAcquisitionFailureWithoutOutput) {
    std::ostringstream out;
    std::ostringstream errors;
    EXPECT_NE(writeFusionReport(acquireScenario(SimulationNanoseconds{1}), out, errors), 0);
    EXPECT_TRUE(out.str().empty());
    EXPECT_EQ(errors.str(), "Fusion demo failed: stage=Acquisition error=AcquisitionError::Sampling"
                            " cause=SamplingError::MissedSample epoch=0 elapsed_ns=10000000\n");
}

TEST(FusionReportTest, ReportsTypedEvaluationFailureWithoutOutput) {
    auto acquisition = acquireScenario();
    ASSERT_TRUE(std::holds_alternative<AcquisitionHistory>(acquisition));
    auto& slot = std::get<AcquisitionHistory>(acquisition).batches.back()[0];
    const auto& gps = std::get<GpsMeasurement>(*slot);
    auto timestamp = gps.acquisition().timestamp();
    ++timestamp.epoch;
    const auto metadata = MeasurementAcquisition::create("gps", timestamp);
    ASSERT_TRUE(metadata);
    const auto replacement = GpsMeasurement::create(
        *metadata, gps.positionWorldM(), gps.positionCovarianceWorldM2());
    ASSERT_TRUE(replacement);
    slot = *replacement;

    std::ostringstream out;
    std::ostringstream errors;
    EXPECT_NE(writeFusionReport(std::move(acquisition), out, errors), 0);
    EXPECT_TRUE(out.str().empty());
    EXPECT_EQ(errors.str(), "Fusion demo failed: stage=AcquisitionAlignment"
                            " error=EvaluationError::EpochMismatch epoch=1 elapsed_ns=4000000000 sensor_id=gps\n");
}

} // namespace
} // namespace aerotwin::fusion_demo
