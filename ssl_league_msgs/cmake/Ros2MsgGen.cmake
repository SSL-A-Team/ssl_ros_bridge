# Ros2MsgGen.cmake
#
# Provides generate_ros2_msgs() — runs the protoc ros2msg plugin once at
# CMake configure time to determine the .msg file list (required up front
# by rosidl_generate_interfaces()), and registers a build-time
# add_custom_command that reruns protoc whenever a proto or plugin script
# changes, so `ninja`/`make` alone regenerates content without a
# reconfigure. If the message *list* itself changes, a build-time check
# fails loudly telling you to reconfigure.
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
cmake_minimum_required(VERSION 3.18)  # CMAKE_CURRENT_FUNCTION_LIST_DIR (3.17), find_program(REQUIRED) (3.18)

include("${CMAKE_CURRENT_LIST_DIR}/AteamProtoGenCommon.cmake")

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
  set(_CMAKE_DIR "${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
  set(_PLUGIN_SRC "${_CMAKE_DIR}/protoc_gen_ros2msg.py")

  ateam_require_script("${_PLUGIN_SRC}" "generate_ros2_msgs")

  file(GLOB _PLUGIN_DEPS "${_CMAKE_DIR}/*.py")

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
    ateam_require_sidecar("${_ARG_SIDECAR}" "generate_ros2_msgs")
    string(APPEND _plugin_opt ",sidecar=${_ARG_SIDECAR}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_ARG_SIDECAR}")
  endif()

  set(_protoc_command
    "${_PROTOC}"
    "--plugin=protoc-gen-ros2msg=${_PLUGIN_WRAPPER}"
    "--ros2msg_opt=${_plugin_opt}"
    "--ros2msg_out=${_ARG_OUTPUT_DIR}/msg"
    ${_proto_path_args}
    ${_ARG_PROTO_FILES}
  )

  # Run once now so the generated .msg file *list* is known at configure
  # time — rosidl_generate_interfaces() requires it up front.
  ateam_run_generator(COMMAND ${_protoc_command} ERROR_PREFIX "generate_ros2_msgs: protoc")

  file(GLOB _generated_abs "${_ARG_OUTPUT_DIR}/msg/*.msg")
  if(NOT _generated_abs)
    message(FATAL_ERROR
      "generate_ros2_msgs: no .msg files found in ${_ARG_OUTPUT_DIR}/msg after generation"
    )
  endif()

  set(_depends ${_ARG_PROTO_FILES} ${_PLUGIN_DEPS})
  if(_ARG_SIDECAR)
    list(APPEND _depends "${_ARG_SIDECAR}")
  endif()

  # Snapshot the expected file list so a build-time check can catch it
  # going stale (see CheckGeneratedMsgList.cmake).
  set(_manifest "${_ARG_OUTPUT_DIR}/.expected_msgs.txt")
  set(_expected_names)
  foreach(_f ${_generated_abs})
    get_filename_component(_n "${_f}" NAME)
    list(APPEND _expected_names "${_n}")
  endforeach()
  list(SORT _expected_names)
  string(REPLACE ";" "\n" _manifest_contents "${_expected_names}")
  file(WRITE "${_manifest}" "${_manifest_contents}\n")

  # Re-run this command at build time (no reconfigure needed) whenever a
  # proto file or plugin script changes, so incremental `ninja`/`make`
  # builds pick up content changes. The check afterward catches the case
  # where the message *list* itself changed (new/removed/renamed message)
  # and rosidl_generate_interfaces() now has a stale file list — that
  # still requires a reconfigure, so fail loudly instead of building
  # silently against old paths.
  add_custom_command(
    OUTPUT ${_generated_abs}
    COMMAND ${_protoc_command}
    COMMAND "${CMAKE_COMMAND}"
      "-DOUTPUT_DIR=${_ARG_OUTPUT_DIR}"
      "-DMANIFEST=${_manifest}"
      -P "${_CMAKE_DIR}/CheckGeneratedMsgList.cmake"
    DEPENDS ${_depends}
    COMMENT "Regenerating ROS2 msg files for ${PROJECT_NAME} from proto sources"
    VERBATIM
  )

  # Full reconfigure still required if the message *list* changes, since
  # that changes what OUTPUT/rosidl need to know about up front.
  set_property(
    DIRECTORY APPEND PROPERTY
    CMAKE_CONFIGURE_DEPENDS ${_depends}
  )

  file(GLOB _generated RELATIVE "${_ARG_OUTPUT_DIR}" "${_ARG_OUTPUT_DIR}/msg/*.msg")
  list(TRANSFORM _generated PREPEND "${_ARG_OUTPUT_DIR}:")

  set(GENERATED_ROS2_MSGS "${_generated}" PARENT_SCOPE)
endfunction()
