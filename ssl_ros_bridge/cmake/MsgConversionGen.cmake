# MsgConversionGen.cmake
#
# Provides generate_message_conversion() — runs gen_message_conversion.py at
# CMake configure time and emits message_conversion_generated.{hpp,cpp}.
#
# Usage:
#   generate_message_conversion(
#     PROTO_FILES  path/to/a.proto path/to/b.proto ...
#     PROTO_PATHS  path/to/proto/include/dir ...
#     SIDECAR      path/to/annotations.json       # optional
#     OUTPUT_DIR   ${CMAKE_CURRENT_BINARY_DIR}/generated_conversion
#   )
#   # Generated files live in OUTPUT_DIR and must be added to a target manually:
#   add_library(mylib ... ${OUTPUT_DIR}/message_conversion_generated.cpp)
#   target_include_directories(mylib PUBLIC ${OUTPUT_DIR})
#
# CMake re-runs automatically when any PROTO_FILES or the SIDECAR changes.

cmake_minimum_required(VERSION 3.16)

function(generate_message_conversion)
  cmake_parse_arguments(_ARG "" "OUTPUT_DIR;SIDECAR" "PROTO_FILES;PROTO_PATHS" ${ARGN})

  if(NOT _ARG_PROTO_FILES)
    message(FATAL_ERROR "generate_message_conversion: PROTO_FILES is required")
  endif()
  if(NOT _ARG_OUTPUT_DIR)
    message(FATAL_ERROR "generate_message_conversion: OUTPUT_DIR is required")
  endif()

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_program(_PROTOC protoc REQUIRED DOC "protoc compiler")

  get_filename_component(_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
  set(_SCRIPT "${_CMAKE_DIR}/gen_message_conversion.py")

  if(NOT EXISTS "${_SCRIPT}")
    message(FATAL_ERROR "generate_message_conversion: script not found at ${_SCRIPT}")
  endif()

  # Build --proto-paths args
  set(_path_args)
  foreach(_p ${_ARG_PROTO_PATHS})
    list(APPEND _path_args "--proto-paths" "${_p}")
  endforeach()

  # Build --sidecar arg
  set(_sidecar_arg)
  if(_ARG_SIDECAR)
    if(NOT EXISTS "${_ARG_SIDECAR}")
      message(FATAL_ERROR "generate_message_conversion: SIDECAR not found: ${_ARG_SIDECAR}")
    endif()
    set(_sidecar_arg "--sidecar" "${_ARG_SIDECAR}")
  endif()

  file(MAKE_DIRECTORY "${_ARG_OUTPUT_DIR}")

  execute_process(
    COMMAND
      "${Python3_EXECUTABLE}" "${_SCRIPT}"
      "--proto-files" ${_ARG_PROTO_FILES}
      ${_path_args}
      ${_sidecar_arg}
      "--output-dir" "${_ARG_OUTPUT_DIR}"
    RESULT_VARIABLE _result
    OUTPUT_VARIABLE _stdout
    ERROR_VARIABLE  _stderr
  )

  if(NOT _result EQUAL 0)
    message(FATAL_ERROR
      "generate_message_conversion: generator failed (exit ${_result}):\n${_stderr}")
  endif()

  # Re-configure when proto files or sidecar change
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${_ARG_PROTO_FILES}
    ${_ARG_SIDECAR}
  )
endfunction()
