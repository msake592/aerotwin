#include <aerotwin/estimation/GpsKalmanUpdater.hpp>
#include <aerotwin/estimation/KalmanPredictor.hpp>
#include <aerotwin/sensors/GpsSensor.hpp>
#include <aerotwin/validation/EstimationMetrics.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace aerotwin;

VehicleState truth(const std::string& motion, double seconds, SimulationTimestamp label) {
    Vector3 velocity = Vector3::Zero();
    Vector3 acceleration = Vector3::Zero();
    if (motion == "cv") {
        velocity = Vector3(2, 1, 0.5);
    } else if (motion == "accelerating") {
        velocity = Vector3(-4, 0, 0);
        acceleration = Vector3(2, 0, 0);
    } else if (motion != "stationary") {
        throw std::invalid_argument("unknown motion");
    }
    return VehicleState::create(
        Vector3(10, -5, 2) + velocity * seconds + 0.5 * acceleration * seconds * seconds,
        Quaternion::Identity(), velocity + acceleration * seconds, Vector3::Zero(), label).value();
}

void run(const std::string& motion, const std::string& mode, long long error, unsigned seed) {
    const std::array<std::string, 9> modes{
        "baseline", "common", "stream", "jitter", "drift", "acquisition", "latency", "oos", "score_only"};
    if (std::find(modes.begin(), modes.end(), mode) == modes.end() ||
        error < -100'000'000 || error > 100'000'000 ||
        (mode == "drift" && std::abs(error) > 25000) ||
        ((mode == "latency" || mode == "oos") && error < 0)) {
        throw std::invalid_argument("invalid mode or error range");
    }
    struct Event {
        int index;
        long long label;
        long long delivery;
        Vector3 position;
    };
    std::vector<Event> events;
    GpsSensor gps(GpsSensorConfig::create("gps", Eigen::Matrix3d::Identity(), seed).value());
    std::mt19937_64 jitter(seed + 10000);
    long long minimum = 0, maximum = 0;
    for (int index = 0; index <= 4; ++index) {
        const long long nominal = index * 1'000'000'000LL;
        const SimulationTimestamp time{0, SimulationNanoseconds{nominal}};
        const auto reference = truth(motion, index, time);
        const Vector3 noise =
            gps.measure(reference, time).value().positionWorldM() - reference.positionWorldM();
        long long perturbation = 0;
        if (index != 0) {
            if (mode == "stream" || mode == "acquisition") {
                perturbation = error;
            }
            if (mode == "jitter") {
                const auto bound = static_cast<unsigned long long>(std::abs(error));
                perturbation = static_cast<long long>(jitter() % (2 * bound + 1)) -
                               static_cast<long long>(bound);
            }
            if (mode == "drift") {
                perturbation = nominal * error / 1'000'000;
            }
        }
        minimum = std::min(minimum, perturbation);
        maximum = std::max(maximum, perturbation);
        long long label = nominal + perturbation;
        if (mode == "baseline" || mode == "latency" || mode == "oos") {
            label = nominal;
        }
        if (mode == "common") {
            const long long origin = 1'000'000'000'000'000'000LL + error;
            label = (origin + nominal) - origin;
        }
        double acquisition = index;
        if (mode == "acquisition") {
            acquisition += perturbation * 1e-9;
            label = nominal;
        }
        long long delivery = mode == "acquisition" ? nominal + perturbation : nominal;
        if (mode == "latency") {
            delivery += error;
        }
        if (mode == "oos" && index == 1) {
            delivery += 2'000'000'000LL + error;
        }
        events.push_back({index, label, delivery,
                          truth(motion, acquisition, time).positionWorldM() + noise});
    }
    std::stable_sort(events.begin(), events.end(), [](const auto& a, const auto& b) {
        return a.delivery < b.delivery;
    });
    EstimatorCovariance6 covariance =
        (EstimatorVector6() << 100, 100, 100, 4, 4, 4).finished().asDiagonal();
    auto estimate = std::get<EstimatorState>(
        EstimatorState::create(EstimatorVector6::Zero(), covariance, {}));
    std::vector<EstimateTruthPair> pairs;
    double nis = 0, nees = 0, max_position = 0, trace_position = 0, trace_velocity = 0;
    double min_eigenvalue = 1e300, symmetry_error = 0;
    double max_estimated_speed = 0, max_truth_speed = 0;
    int rejected = 0, ordering = 0;
    for (const auto& event : events) {
        const SimulationTimestamp label{0, SimulationNanoseconds{event.label}};
        auto prior = estimate;
        if (!pairs.empty()) {
            auto prediction =
                KalmanPredictor::predict(estimate, label, 0.01 * Eigen::Matrix3d::Identity());
            if (!std::holds_alternative<EstimatorState>(prediction)) {
                ++rejected;
                ++ordering;
                continue;
            }
            prior = std::get<EstimatorState>(prediction);
        }
        auto measurement = GpsMeasurement::create(
            MeasurementAcquisition::create("gps", label).value(), event.position,
            Eigen::Matrix3d::Identity()).value();
        auto updated = GpsKalmanUpdater::update(prior, measurement);
        if (!std::holds_alternative<EstimatorState>(updated)) {
            ++rejected;
            continue;
        }
        nis += std::get<ConsistencyScore>(computeGpsNis(prior, measurement)).value;
        estimate = std::get<EstimatorState>(updated);
        // Explicit evaluation-only timestamp projection for clock-label errors.
        // Stream skew instead scores at the assigned physical event time.
        const long long scoring_ns = mode == "stream" ? event.label : event.index * 1'000'000'000LL;
        const SimulationTimestamp scoring_time{0, SimulationNanoseconds{scoring_ns}};
        auto scoring_estimate = std::get<EstimatorState>(
            EstimatorState::create(estimate.vector(), estimate.covariance(), scoring_time));
        auto reference = truth(motion, scoring_ns * 1e-9, scoring_time);
        if (mode == "score_only") {
            reference = VehicleState::create(
                reference.positionWorldM() + Vector3(100, 0, 0), Quaternion::Identity(),
                reference.linearVelocityWorldMps(), Vector3::Zero(), scoring_time).value();
        }
        pairs.push_back({scoring_estimate, reference});
        nees += std::get<ConsistencyScore>(
                    computeNees(scoring_estimate, reference, NeesComponents::FullState))
                    .value;
        max_position = std::max(
            max_position, (estimate.positionWorldM() - reference.positionWorldM()).norm());
        max_estimated_speed = std::max(max_estimated_speed, estimate.velocityWorldMps().norm());
        max_truth_speed = std::max(max_truth_speed, reference.linearVelocityWorldMps().norm());
        const auto posterior = estimate.covariance();
        trace_position += posterior.topLeftCorner<3, 3>().trace();
        trace_velocity += posterior.bottomRightCorner<3, 3>().trace();
        min_eigenvalue = std::min(min_eigenvalue,
            Eigen::SelfAdjointEigenSolver<EstimatorCovariance6>(posterior).eigenvalues().minCoeff());
        symmetry_error = std::max(
            symmetry_error, (posterior - posterior.transpose()).cwiseAbs().maxCoeff());
    }
    const auto rmse = std::get<RmseResult>(computeRmse(pairs));
    const double count = static_cast<double>(pairs.size());
    std::cout << std::setprecision(17)
              << "{\"position_rmse\":" << rmse.position_3d_m
              << ",\"velocity_rmse\":" << rmse.velocity_3d_mps
              << ",\"max_position_error\":" << max_position
              << ",\"mean_nis\":" << nis / count
              << ",\"mean_nees\":" << nees / count
              << ",\"mean_position_trace\":" << trace_position / count
              << ",\"mean_velocity_trace\":" << trace_velocity / count
              << ",\"minimum_covariance_eigenvalue\":" << min_eigenvalue
              << ",\"symmetry_error\":" << symmetry_error
              << ",\"max_estimated_speed\":" << max_estimated_speed
              << ",\"max_truth_speed\":" << max_truth_speed
              << ",\"accepted\":" << pairs.size() << ",\"rejected\":" << rejected
              << ",\"ordering_violations\":" << ordering
              << ",\"min_perturbation_ns\":" << minimum
              << ",\"max_perturbation_ns\":" << maximum << "}\n";
}

int main() {
    try {
        std::string motion, mode;
        long long error;
        long long seed;
        std::string line;
        while (std::getline(std::cin, line)) {
            std::istringstream input(line);
            std::string extra;
            if (!(input >> motion >> mode >> error >> seed) || input >> extra || seed < 0 ||
                seed > std::numeric_limits<unsigned>::max()) {
                throw std::invalid_argument("malformed input");
            }
            run(motion, mode, error, static_cast<unsigned>(seed));
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
