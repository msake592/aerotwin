include(FetchContent)

# Prefer extraction-time timestamps where supported (CMake 3.24+).
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# Eigen is header-only. Skip its standalone project so its tests, benchmarks,
# examples, and compiler settings do not enter the AeroTwin build.
FetchContent_Declare(eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    SOURCE_SUBDIR aerotwin-header-only
)
FetchContent_MakeAvailable(eigen)
add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}"
)

add_library(aerotwin_core STATIC
    src/common/Version.cpp
    src/math/Rotations.cpp
    src/math/Transformations.cpp
    src/simulation/SimulationClock.cpp
    src/simulation/VehicleState.cpp
    src/simulation/KinematicIntegrator.cpp
    src/simulation/ScenarioRunner.cpp
    src/telemetry/TruthTelemetry.cpp
    src/sensors/Measurements.cpp
    src/sensors/Covariance.cpp
    src/sensors/GaussianNoise.cpp
    src/sensors/GpsSensor.cpp
    src/sensors/ImuSensor.cpp
    src/sensors/SensorSamplingPipeline.cpp
    src/estimation/EstimatorState.cpp
)

target_include_directories(aerotwin_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(aerotwin_core PUBLIC Eigen3::Eigen)
target_compile_features(aerotwin_core PUBLIC cxx_std_20)
target_compile_options(aerotwin_core PRIVATE
    $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
