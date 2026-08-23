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

cmake_minimum_required(VERSION 3.18)  # CMAKE_CURRENT_FUNCTION_LIST_DIR (3.17), find_program(REQUIRED) (3.18)

include("${CMAKE_CURRENT_LIST_DIR}/../../ssl_league_msgs/cmake/ProtoGenShared.cmake")

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

  set(_CMAKE_DIR "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
  set(_SCRIPT "${_CMAKE_DIR}/gen_message_conversion.py")
  set(_SHARED_SCRIPT "${_CMAKE_DIR}/../../ssl_league_msgs/cmake/proto_shared.py")

  protogen_require_script("${_SCRIPT}" "generate_message_conversion")
  protogen_require_script("${_SHARED_SCRIPT}" "generate_message_conversion")

  # Build --proto-paths args
  set(_path_args)
  foreach(_p ${_ARG_PROTO_PATHS})
    list(APPEND _path_args "--proto-paths" "${_p}")
  endforeach()

  # Build --sidecar arg
  set(_sidecar_arg)
  if(_ARG_SIDECAR)
    protogen_require_annotation_file("${_ARG_SIDECAR}" "generate_message_conversion")
    set(_sidecar_arg "--sidecar" "${_ARG_SIDECAR}")
  endif()

  file(MAKE_DIRECTORY "${_ARG_OUTPUT_DIR}")

  set(_gen_command
    "${Python3_EXECUTABLE}" "${_SCRIPT}"
    "--proto-files" ${_ARG_PROTO_FILES}
    ${_path_args}
    ${_sidecar_arg}
    "--output-dir" "${_ARG_OUTPUT_DIR}"
    "--protoc-path" "${_PROTOC}"
  )

  set(_hpp "${_ARG_OUTPUT_DIR}/message_conversion_generated.hpp")
  set(_cpp "${_ARG_OUTPUT_DIR}/message_conversion_generated.cpp")

  # Run once now so the generated files exist for configure-time consumers
  # (e.g. add_library() argument lists).
  protogen_run_generator(COMMAND ${_gen_command} ERROR_PREFIX "generate_message_conversion: generator")

  set(_depends ${_ARG_PROTO_FILES} "${_SCRIPT}" "${_SHARED_SCRIPT}")
  if(_ARG_SIDECAR)
    list(APPEND _depends "${_ARG_SIDECAR}")
  endif()

  # Rerun at build time (no reconfigure needed) whenever a proto, the
  # generator script, or the sidecar changes.
  add_custom_command(
    OUTPUT "${_hpp}" "${_cpp}"
    COMMAND ${_gen_command}
    DEPENDS ${_depends}
    COMMENT "Regenerating message conversion code for ${PROJECT_NAME} from proto sources"
    VERBATIM
  )

  # Full reconfigure still required if the output file set itself would
  # need to change (it doesn't here — always exactly these two files).
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_depends})
endfunction()
