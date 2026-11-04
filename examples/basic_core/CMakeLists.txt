add_executable(aerotwin_basic_core main.cpp)
target_link_libraries(aerotwin_basic_core PRIVATE aerotwin_core)
target_compile_options(aerotwin_basic_core PRIVATE
    $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
