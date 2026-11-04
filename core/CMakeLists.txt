add_library(aerotwin_core STATIC
    src/common/Version.cpp
)

target_include_directories(aerotwin_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_compile_features(aerotwin_core PUBLIC cxx_std_20)
target_compile_options(aerotwin_core PRIVATE
    $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
