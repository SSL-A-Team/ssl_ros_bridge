# Ros2MsgGen.cmake
#
# Provides generate_ros2_msgs() — runs the protoc ros2msg plugin at CMake
# configure time and returns the list of generated .msg files.
#
# Usage:
#   generate_ros2_msgs(
#     PROTO_FILES   path/to/a.proto path/to/b.proto ...
#     PROTO_PATHS   path/to/proto/include/dir ...   # --proto_path roots
#     OUTPUT_DIR    ${CMAKE_CURRENT_BINARY_DIR}/msg  # default if omitted
#     OPTIONAL_SUBMSG  HAS_FIELD                     # or ERROR; default HAS_FIELD
#     SIDECAR          path/to/annotations.json       # optional
#   )
#   # After the call, ${GENERATED_ROS2_MSGS} contains the .msg file list.
#   rosidl_generate_interfaces(${PROJECT_NAME} ${GENERATED_ROS2_MSGS})
#
# OPTIONAL_SUBMSG controls handling of non-oneof message-type fields:
#   HAS_FIELD  (default) Emit a bool has_<field> presence sentinel.
#   ERROR      Fail the build if any such field exists, forcing the schema
#              author to either move it into a oneof or switch to HAS_FIELD.
#
# SIDECAR (optional) path to a JSON annotation file. Fields listed in the
#   sidecar are consumed and emitted as annotated ROS types; remaining fields
#   pass through normally. See protoc_gen_ros2msg.py docstring for schema.
#
# Generation runs at configure time so .msg files exist when
# rosidl_generate_interfaces() is called.  CMake re-runs automatically when
# any PROTO_FILES changes (CMAKE_CONFIGURE_DEPENDS).

cmake_minimum_required(VERSION 3.16)

function(generate_ros2_msgs)
  cmake_parse_arguments(
    _ARG
    ""
    "OUTPUT_DIR;OPTIONAL_SUBMSG;SIDECAR"
    "PROTO_FILES;PROTO_PATHS"
    ${ARGN}
  )

  # --- Validate arguments ---
  if(NOT _ARG_PROTO_FILES)
    message(FATAL_ERROR "generate_ros2_msgs: PROTO_FILES is required")
  endif()

  # --- Defaults ---
  if(NOT _ARG_OUTPUT_DIR)
    set(_ARG_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/ros2_msgs")
  endif()

  if(NOT _ARG_OPTIONAL_SUBMSG)
    set(_ARG_OPTIONAL_SUBMSG "HAS_FIELD")
  endif()
  string(TOLOWER "${_ARG_OPTIONAL_SUBMSG}" _opt_submsg)

  if(NOT _opt_submsg STREQUAL "has_field" AND NOT _opt_submsg STREQUAL "error")
    message(FATAL_ERROR
      "generate_ros2_msgs: OPTIONAL_SUBMSG must be HAS_FIELD or ERROR, "
      "got '${_ARG_OPTIONAL_SUBMSG}'"
    )
  endif()

  # --- Find tools ---
  find_program(_PROTOC protoc REQUIRED
    DOC "protoc compiler (install via nix: protobuf)"
  )
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # --- Plugin path (sibling of this .cmake file) ---
  get_filename_component(_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
  set(_PLUGIN_SRC "${_CMAKE_DIR}/protoc_gen_ros2msg.py")

  if(NOT EXISTS "${_PLUGIN_SRC}")
    message(FATAL_ERROR
      "generate_ros2_msgs: plugin not found at ${_PLUGIN_SRC}"
    )
  endif()

  # Generate an executable wrapper in the build tree so we can pass an
  # explicit Python interpreter without relying on the script's shebang.
  set(_PLUGIN_WRAPPER "${CMAKE_BINARY_DIR}/protoc_gen_ros2msg")
  file(WRITE "${_PLUGIN_WRAPPER}"
    "#!/bin/sh\nset -e\nexec \"${Python3_EXECUTABLE}\" \"${_PLUGIN_SRC}\" \"$@\"\n"
  )
  file(CHMOD "${_PLUGIN_WRAPPER}"
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )

  # --- Output directory ---
  file(MAKE_DIRECTORY "${_ARG_OUTPUT_DIR}/msg")

  # --- Build --proto_path arguments ---
  set(_proto_path_args)
  foreach(_path ${_ARG_PROTO_PATHS})
    list(APPEND _proto_path_args "--proto_path=${_path}")
  endforeach()

  # --- Build plugin options ---
  set(_plugin_opt "optional_submsg=${_opt_submsg}")
  if(_ARG_SIDECAR)
    if(NOT EXISTS "${_ARG_SIDECAR}")
      message(FATAL_ERROR "generate_ros2_msgs: SIDECAR file not found: ${_ARG_SIDECAR}")
    endif()
    string(APPEND _plugin_opt ",sidecar=${_ARG_SIDECAR}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_ARG_SIDECAR}")
  endif()

  # --- Run protoc at configure time ---
  # TODO convert this to add_custom_command
  execute_process(
    COMMAND
      "${_PROTOC}"
      "--plugin=protoc-gen-ros2msg=${_PLUGIN_WRAPPER}"
      "--ros2msg_opt=${_plugin_opt}"
      "--ros2msg_out=${_ARG_OUTPUT_DIR}/msg"
      ${_proto_path_args}
      ${_ARG_PROTO_FILES}
    RESULT_VARIABLE _result
    ERROR_VARIABLE  _stderr
    OUTPUT_QUIET
  )

  if(NOT _result EQUAL 0)
    message(FATAL_ERROR
      "generate_ros2_msgs: protoc failed (exit ${_result}):\n${_stderr}"
    )
  endif()

  # Re-run CMake configure when any proto file changes.
  set_property(
    DIRECTORY APPEND PROPERTY
    CMAKE_CONFIGURE_DEPENDS ${_ARG_PROTO_FILES}
  )

  # Collect results and expose to caller.
  file(GLOB _generated RELATIVE ${_ARG_OUTPUT_DIR} "${_ARG_OUTPUT_DIR}/msg/*.msg")
  list(TRANSFORM _generated PREPEND "${_ARG_OUTPUT_DIR}:")
  if(NOT _generated)
    message(FATAL_ERROR
      "generate_ros2_msgs: no .msg files found in ${_ARG_OUTPUT_DIR}/msg after generation"
    )
  endif()

  set(GENERATED_ROS2_MSGS "${_generated}" PARENT_SCOPE)
endfunction()
