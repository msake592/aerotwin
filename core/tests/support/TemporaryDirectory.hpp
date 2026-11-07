#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

namespace aerotwin::test {

class TemporaryDirectory {
public:
    TemporaryDirectory() {
        const auto root = std::filesystem::temp_directory_path();
        for (int attempt = 0; attempt < 1000; ++attempt) {
            auto candidate = root / ("aerotwin-test-" + std::to_string(attempt));
            if (std::filesystem::create_directory(candidate)) {
                path_ = std::move(candidate);
                return;
            }
        }
        throw std::runtime_error("Could not reserve a temporary test directory");
    }

    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    TemporaryDirectory(const TemporaryDirectory&) = delete;
    TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

} // namespace aerotwin::test
