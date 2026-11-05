include(FetchContent)

# Prefer extraction-time timestamps where supported (CMake 3.24+).
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

set(BUILD_GMOCK OFF CACHE BOOL "Build GoogleMock" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "Install GoogleTest" FORCE)
if(MSVC)
    set(gtest_force_shared_crt ON CACHE BOOL "Use the shared MSVC runtime" FORCE)
endif()

FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.18.0.tar.gz
    URL_HASH SHA256=6e3191c1455468b3fc35a417fb565c1c5071aee1b7e7f85e30cf48a98d37d8b5
)
FetchContent_MakeAvailable(googletest)

add_executable(aerotwin_core_tests
    unit/VersionTest.cpp
    unit/EigenTest.cpp
)
target_link_libraries(aerotwin_core_tests PRIVATE aerotwin_core GTest::gtest_main)
target_compile_options(aerotwin_core_tests PRIVATE
    $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

include(GoogleTest)
gtest_discover_tests(aerotwin_core_tests)
