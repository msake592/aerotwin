add_executable(aerotwin_motion_demo main.cpp)
target_link_libraries(aerotwin_motion_demo PRIVATE aerotwin_core)
target_compile_options(aerotwin_motion_demo PRIVATE
    $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
