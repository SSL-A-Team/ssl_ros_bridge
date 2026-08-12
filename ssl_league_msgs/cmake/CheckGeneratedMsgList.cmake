# Invoked via `${CMAKE_COMMAND} -P` as a build-time step after protoc
# regenerates .msg files. rosidl_generate_interfaces() was told a fixed
# file list at the last CMake configure; if protoc now emits a different
# set of files (message added/removed in a .proto), that list is stale
# and only a reconfigure will fix it. Fail loudly rather than silently
# building against the old list.
#
# Args (via -D): OUTPUT_DIR, MANIFEST

file(GLOB _actual "${OUTPUT_DIR}/msg/*.msg")
set(_actual_names)
foreach(_f ${_actual})
  get_filename_component(_n "${_f}" NAME)
  list(APPEND _actual_names "${_n}")
endforeach()
list(SORT _actual_names)

file(STRINGS "${MANIFEST}" _expected_names)
list(SORT _expected_names)

if(NOT _actual_names STREQUAL _expected_names)
  message(FATAL_ERROR
    "Generated .msg file list changed since the last CMake configure "
    "(a message was likely added/removed/renamed in a .proto). "
    "rosidl_generate_interfaces() was called with a stale file list. "
    "Re-run CMake configure (e.g. `colcon build --cmake-force-configure`) "
    "and build again.\n"
    "  expected: ${_expected_names}\n"
    "  actual:   ${_actual_names}"
  )
endif()
