if(DEFINED PROBE)
    execute_process(COMMAND "${PROBE}" "${DEMO}" "${MODE}"
        RESULT_VARIABLE status ERROR_VARIABLE errors)
    if(NOT status STREQUAL "1" OR
       NOT errors STREQUAL "Fusion demo failed: could not write CSV to stdout\n")
        message(FATAL_ERROR "Expected stdout write failure for ${MODE} (status=${status}): ${errors}")
    endif()
    return()
endif()

set(first_path "${CMAKE_CURRENT_BINARY_DIR}/fusion-demo-first.csv")
set(replay_path "${CMAKE_CURRENT_BINARY_DIR}/fusion-demo-replay.csv")

execute_process(COMMAND "${DEMO}" RESULT_VARIABLE status OUTPUT_FILE "${first_path}" ERROR_VARIABLE errors)
if(NOT status STREQUAL "0" OR NOT errors STREQUAL "")
    message(FATAL_ERROR "Fusion demo invocation failed (status=${status}): ${errors}")
endif()
file(STRINGS "${first_path}" header LIMIT_COUNT 1)
if(NOT header MATCHES "^record_type,schema_version,epoch,elapsed_ns,sensor_id,")
    message(FATAL_ERROR "Fusion demo did not produce a CSV header")
endif()

execute_process(COMMAND "${DEMO}" RESULT_VARIABLE status OUTPUT_FILE "${replay_path}" ERROR_VARIABLE errors)
if(NOT status STREQUAL "0" OR NOT errors STREQUAL "")
    message(FATAL_ERROR "Fusion demo replay failed (status=${status}): ${errors}")
endif()
file(SHA256 "${first_path}" first_hash)
file(SHA256 "${replay_path}" replay_hash)
if(NOT first_hash STREQUAL replay_hash)
    message(FATAL_ERROR "Fusion demo replay produced different CSV bytes")
endif()
file(REMOVE "${first_path}" "${replay_path}")

execute_process(COMMAND "${DEMO}" unexpected-argument
    RESULT_VARIABLE status OUTPUT_VARIABLE output ERROR_VARIABLE errors)
if(NOT status STREQUAL "1" OR NOT output STREQUAL "" OR
   NOT errors MATCHES "Usage: aerotwin_fusion_demo")
    message(FATAL_ERROR "Expected a usage error with empty stdout (status=${status}): ${errors}")
endif()
