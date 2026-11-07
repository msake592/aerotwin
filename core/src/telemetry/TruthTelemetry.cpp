#include "TruthTelemetryInternal.hpp"

#include <cerrno>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <new>
#include <sstream>
#include <utility>
#include <iomanip>
#include <limits>
#include <locale>

namespace aerotwin {
namespace {

class Descriptor {
public:
    explicit Descriptor(int fd) noexcept : fd_(fd) {}
    ~Descriptor() { close(); }

    Descriptor(const Descriptor&) = delete;
    Descriptor& operator=(const Descriptor&) = delete;

    int get() const noexcept { return fd_; }

    bool close() noexcept {
        const int fd = std::exchange(fd_, -1);
        // Retrying close can close a reused descriptor. Treat any error as failure.
        return fd < 0 || ::close(fd) == 0;
    }

private:
    int fd_;
};

bool sameIdentity(const struct stat& left, const struct stat& right) noexcept {
    return left.st_dev == right.st_dev && left.st_ino == right.st_ino;
}

bool matchesEntry(int directory, const char* name, const struct stat& identity) noexcept {
    struct stat current {};
    return ::fstatat(directory, name, &current, AT_SYMLINK_NOFOLLOW) == 0 &&
           sameIdentity(current, identity);
}

class OutputFile {
public:
    OutputFile(int directory, const char* name) noexcept
        : directory_(directory), name_(name),
          fd_(::openat(directory, name, O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
                       0600)),
          owned_(fd_.get() >= 0 && ::fstat(fd_.get(), &identity_) == 0) {}

    ~OutputFile() {
        if (owned_ && matchesEntry(directory_, name_, identity_)) {
            ::unlinkat(directory_, name_, 0);
        }
    }

    bool write(std::string_view bytes) const noexcept {
        if (!owned_) {
            return false;
        }
        while (!bytes.empty()) {
            const auto count = ::write(fd_.get(), bytes.data(), bytes.size());
            if (count < 0 && errno == EINTR) {
                continue;
            }
            if (count <= 0) {
                return false;
            }
            bytes.remove_prefix(static_cast<std::size_t>(count));
        }
        return true;
    }

    bool close() noexcept { return fd_.close(); }
    bool stillLinked() const noexcept {
        return owned_ && matchesEntry(directory_, name_, identity_);
    }
    void keep() noexcept { owned_ = false; }

private:
    int directory_;
    const char* name_;
    Descriptor fd_;
    struct stat identity_ {};
    bool owned_;
};

bool writeSamples(const ScenarioResult& result, OutputFile& output, detail::ExportHook hook,
                  void* context) {
    std::ostringstream csv;
    csv.imbue(std::locale::classic());
    csv << std::setprecision(std::numeric_limits<double>::max_digits10)
        << "epoch,elapsed_ns,position_e_m,position_n_m,position_u_m,"
           "orientation_w,orientation_x,orientation_y,orientation_z,"
           "velocity_e_mps,velocity_n_mps,velocity_u_mps,"
           "angular_velocity_f_radps,angular_velocity_l_radps,angular_velocity_u_radps\n";
    if (!csv || !output.write(csv.view()) ||
        (hook && !hook(detail::ExportCheckpoint::CsvHeaderWritten, context))) {
        return false;
    }
    for (const auto& sample : result.samples()) {
        csv.str("");
        const auto timestamp = sample.timestamp();
        const auto p = sample.positionWorldM();
        const auto q = sample.orientationWorldFromBody();
        const auto v = sample.linearVelocityWorldMps();
        const auto w = sample.angularVelocityBodyRadps();
        csv << timestamp.epoch << ',' << timestamp.elapsed.count() << ','
            << p.x() << ',' << p.y() << ',' << p.z() << ','
            << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ','
            << v.x() << ',' << v.y() << ',' << v.z() << ','
            << w.x() << ',' << w.y() << ',' << w.z() << '\n';
        if (!csv || !output.write(csv.view())) {
            return false;
        }
    }
    return output.close();
}

bool writeMetadata(const ScenarioResult& result, OutputFile& output) {
    const auto& config = result.config();
    std::ostringstream metadata;
    metadata.imbue(std::locale::classic());
    metadata << "schema_version=" << ScenarioResult::kSchemaVersion << '\n'
             << "scenario_id=" << config.identifier().value_or("") << '\n'
             << "seed=" << config.seed() << '\n'
             << "timestep_ns=" << config.timestep().count() << '\n'
             << "step_count=" << config.stepCount() << '\n'
             << "sample_count=" << result.samples().size() << '\n'
             << "world_frame=ENU\nbody_frame=FLU\n"
                "orientation=active_Hamilton_body_to_world_WXYZ\n"
                "time_unit=ns\nposition_unit=m\nlinear_velocity_unit=m/s\n"
                "angular_velocity_unit=rad/s\n";
    return metadata && output.write(metadata.view()) && output.close();
}

} // namespace

bool detail::exportTruthTelemetryImpl(const ScenarioResult& result,
                                      const std::filesystem::path& output_directory,
                                      ExportHook hook, void* context) {
    if (result.samples().size() != static_cast<std::size_t>(result.config().stepCount()) + 1 ||
        output_directory.empty() || output_directory.native().find('\0') != std::string::npos) {
        return false;
    }

    auto destination = output_directory;
    while (!destination.has_filename() && destination != destination.root_path()) {
        destination = destination.parent_path();
    }
    const auto leaf = destination.filename();
    if (leaf.empty() || leaf == "." || leaf == "..") {
        return false;
    }
    const auto parent_path = destination.has_parent_path() ? destination.parent_path() : ".";
    constexpr int directory_flags = O_RDONLY | O_DIRECTORY | O_CLOEXEC;
    const Descriptor parent(::open(parent_path.c_str(), directory_flags));
    struct stat parent_identity {};
    if (parent.get() < 0 || ::fstat(parent.get(), &parent_identity) != 0) {
        return false;
    }
    // mkdirat cannot return a descriptor. In a shared parent, require sticky
    // semantics and a trusted owner to protect the leaf until openat pins it.
    const bool shared = (parent_identity.st_mode & (S_IWGRP | S_IWOTH)) != 0;
    if ((parent_identity.st_uid != ::geteuid() && parent_identity.st_uid != 0) ||
        (shared && (parent_identity.st_mode & S_ISVTX) == 0) ||
        ::mkdirat(parent.get(), leaf.c_str(), 0700) != 0) {
        return false;
    }

    struct stat observed {};
    if (::fstatat(parent.get(), leaf.c_str(), &observed, AT_SYMLINK_NOFOLLOW) != 0 ||
        !S_ISDIR(observed.st_mode) || observed.st_uid != ::geteuid() ||
        (observed.st_mode & 077) != 0) {
        return false;
    }
    const Descriptor directory(::openat(parent.get(), leaf.c_str(), directory_flags | O_NOFOLLOW));
    struct stat identity {};
    if (directory.get() < 0 || ::fstat(directory.get(), &identity) != 0 ||
        !sameIdentity(observed, identity) ||
        (hook && !hook(ExportCheckpoint::DirectoryAcquired, context))) {
        return false;
    }

    // Leave the leaf directory on failure. POSIX has no atomic identity-checked rmdir.
    OutputFile csv(directory.get(), "truth_trajectory.csv");
    if (!writeSamples(result, csv, hook, context)) {
        return false;
    }
    OutputFile metadata(directory.get(), "run_metadata.txt");
    if (!writeMetadata(result, metadata)) {
        return false;
    }

    const Descriptor current_parent(::open(parent_path.c_str(), directory_flags));
    struct stat current_identity {};
    if (current_parent.get() < 0 || ::fstat(current_parent.get(), &current_identity) != 0 ||
        !sameIdentity(parent_identity, current_identity) ||
        !matchesEntry(parent.get(), leaf.c_str(), identity) ||
        !csv.stillLinked() || !metadata.stillLinked()) {
        return false;
    }

    csv.keep();
    metadata.keep();
    return true;
}

bool exportTruthTelemetry(const ScenarioResult& result,
                          const std::filesystem::path& output_directory) {
    try {
        return detail::exportTruthTelemetryImpl(result, output_directory, nullptr, nullptr);
    } catch (const std::bad_alloc&) {
        return false;
    }
}

} // namespace aerotwin
