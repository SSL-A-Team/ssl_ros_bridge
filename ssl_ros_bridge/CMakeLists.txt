cmake_minimum_required(VERSION 3.18)
project(ssl_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ssl_league_msgs REQUIRED)
find_package(ssl_league_protobufs REQUIRED)
find_package(ssl_ros_bridge_msgs REQUIRED)

include(cmake/MsgConversionGen.cmake)

set(_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ssl_league_protobufs/proto")
set(_SIDECAR   "${CMAKE_CURRENT_SOURCE_DIR}/../ssl_league_msgs/cmake/ssl_ros_annotations.json")
set(_CONV_OUT  "${CMAKE_CURRENT_BINARY_DIR}/generated_conversion")

generate_message_conversion(
  PROTO_FILES
    ${_PROTO_DIR}/ssl_gc_common.proto
    ${_PROTO_DIR}/ssl_gc_geometry.proto
    ${_PROTO_DIR}/ssl_gc_game_event.proto
    ${_PROTO_DIR}/ssl_gc_referee_message.proto
    ${_PROTO_DIR}/ssl_gc_rcon.proto
    ${_PROTO_DIR}/ssl_vision_detection.proto
    ${_PROTO_DIR}/ssl_vision_geometry.proto
    ${_PROTO_DIR}/ssl_vision_wrapper.proto
    ${_PROTO_DIR}/ssl_simulation_config.proto
    ${_PROTO_DIR}/ssl_simulation_control.proto
    ${_PROTO_DIR}/ssl_simulation_error.proto
  PROTO_PATHS ${_PROTO_DIR}
  SIDECAR     ${_SIDECAR}
  OUTPUT_DIR  ${_CONV_OUT}
)

add_subdirectory(src/core)
add_subdirectory(src/game_controller_bridge)
add_subdirectory(src/log2bag)
add_subdirectory(src/team_client)
add_subdirectory(src/vision_bridge)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
