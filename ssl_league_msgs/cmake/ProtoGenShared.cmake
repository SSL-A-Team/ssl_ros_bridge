# ProtoGenShared.cmake
#
# Shared plumbing for the two proto-code-generator CMake modules —
# Ros2MsgGen.cmake (ssl_league_msgs, .msg generation) and
# MsgConversionGen.cmake (ssl_ros_bridge, C++ conversion codegen) — so the
# "run a generator script, fail loudly and consistently on error" pattern
# lives once instead of as two hand-copied blocks.
#
# Provides:
#   protogen_require_script(SCRIPT_PATH ERROR_PREFIX)
#     FATAL_ERROR if SCRIPT_PATH doesn't exist.
#
#   protogen_require_annotation_file(ANNOTATION_FILE_PATH ERROR_PREFIX)
#     FATAL_ERROR if ANNOTATION_FILE_PATH is set but doesn't exist. No-op if unset.
#
#   protogen_run_generator(COMMAND <cmd...> ERROR_PREFIX <text>)
#     Runs COMMAND via execute_process(); on nonzero exit, FATAL_ERRORs with
#     ERROR_PREFIX and the captured stderr.

cmake_minimum_required(VERSION 3.18)

function(protogen_require_script SCRIPT_PATH ERROR_PREFIX)
  if(NOT EXISTS "${SCRIPT_PATH}")
    message(FATAL_ERROR "${ERROR_PREFIX}: script not found at ${SCRIPT_PATH}")
  endif()
endfunction()

function(protogen_require_annotation_file ANNOTATION_FILE_PATH ERROR_PREFIX)
  if(ANNOTATION_FILE_PATH AND NOT EXISTS "${ANNOTATION_FILE_PATH}")
    message(FATAL_ERROR "${ERROR_PREFIX}: annotation file not found: ${ANNOTATION_FILE_PATH}")
  endif()
endfunction()

function(protogen_run_generator)
  cmake_parse_arguments(_ARG "" "ERROR_PREFIX" "COMMAND" ${ARGN})
  if(NOT _ARG_COMMAND)
    message(FATAL_ERROR "protogen_run_generator: COMMAND is required")
  endif()
  if(NOT _ARG_ERROR_PREFIX)
    message(FATAL_ERROR "protogen_run_generator: ERROR_PREFIX is required")
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
