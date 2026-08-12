# AteamProtoGenCommon.cmake
#
# Shared plumbing for the two proto-code-generator CMake modules —
# Ros2MsgGen.cmake (ssl_league_msgs, .msg generation) and
# MsgConversionGen.cmake (ssl_ros_bridge, C++ conversion codegen) — so the
# "run a generator script, fail loudly and consistently on error" pattern
# lives once instead of as two hand-copied blocks.
#
# Provides:
#   ateam_require_script(SCRIPT_PATH ERROR_PREFIX)
#     FATAL_ERROR if SCRIPT_PATH doesn't exist.
#
#   ateam_require_sidecar(SIDECAR_PATH ERROR_PREFIX)
#     FATAL_ERROR if SIDECAR_PATH is set but doesn't exist. No-op if unset.
#
#   ateam_run_generator(COMMAND <cmd...> ERROR_PREFIX <text>)
#     Runs COMMAND via execute_process(); on nonzero exit, FATAL_ERRORs with
#     ERROR_PREFIX and the captured stderr.

cmake_minimum_required(VERSION 3.18)

function(ateam_require_script SCRIPT_PATH ERROR_PREFIX)
  if(NOT EXISTS "${SCRIPT_PATH}")
    message(FATAL_ERROR "${ERROR_PREFIX}: script not found at ${SCRIPT_PATH}")
  endif()
endfunction()

function(ateam_require_sidecar SIDECAR_PATH ERROR_PREFIX)
  if(SIDECAR_PATH AND NOT EXISTS "${SIDECAR_PATH}")
    message(FATAL_ERROR "${ERROR_PREFIX}: SIDECAR file not found: ${SIDECAR_PATH}")
  endif()
endfunction()

function(ateam_run_generator)
  cmake_parse_arguments(_ARG "" "ERROR_PREFIX" "COMMAND" ${ARGN})
  if(NOT _ARG_COMMAND)
    message(FATAL_ERROR "ateam_run_generator: COMMAND is required")
  endif()
  if(NOT _ARG_ERROR_PREFIX)
    message(FATAL_ERROR "ateam_run_generator: ERROR_PREFIX is required")
  endif()

  execute_process(
    COMMAND ${_ARG_COMMAND}
    RESULT_VARIABLE _result
    ERROR_VARIABLE  _stderr
    OUTPUT_QUIET
  )

  if(NOT _result EQUAL 0)
    message(FATAL_ERROR "${_ARG_ERROR_PREFIX} failed (exit ${_result}):\n${_stderr}")
  endif()
endfunction()
