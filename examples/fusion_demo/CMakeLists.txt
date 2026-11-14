add_library(aerotwin_fusion_acquisition STATIC ScenarioAcquisition.cpp FusionEvaluation.cpp FusionReport.cpp)
target_link_libraries(aerotwin_fusion_acquisition PUBLIC aerotwin_core)
target_include_directories(aerotwin_fusion_acquisition PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(aerotwin_fusion_acquisition PRIVATE
    $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:-Wall;-Wextra;-Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(aerotwin_fusion_demo main.cpp)
target_link_libraries(aerotwin_fusion_demo PRIVATE aerotwin_fusion_acquisition)

if(BUILD_TESTING)
    add_test(NAME FusionDemoExecutable.SmokeReplayAndUsage
        COMMAND "${CMAKE_COMMAND}"
            "-DDEMO=$<TARGET_FILE:aerotwin_fusion_demo>"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/CheckFusionDemo.cmake"
    )

    if(UNIX)
        add_executable(aerotwin_fusion_output_failure_probe OutputFailureProbe.cpp)
        foreach(mode IN ITEMS read-only closed-pipe)
            add_test(NAME FusionDemoExecutable.${mode}
                COMMAND "${CMAKE_COMMAND}"
                    "-DDEMO=$<TARGET_FILE:aerotwin_fusion_demo>"
                    "-DPROBE=$<TARGET_FILE:aerotwin_fusion_output_failure_probe>"
                    "-DMODE=${mode}"
                    -P "${CMAKE_CURRENT_SOURCE_DIR}/CheckFusionDemo.cmake"
            )
        endforeach()
    endif()
endif()
