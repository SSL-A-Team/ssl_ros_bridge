cmake_minimum_required(VERSION 3.16)
project(ssl_league_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include(cmake/Ros2MsgGen.cmake)

set(_PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ssl_league_protobufs/proto")

generate_ros2_msgs(
  PROTO_FILES
    ${_PROTO_DIR}/ssl_gc_common.proto
    ${_PROTO_DIR}/ssl_gc_geometry.proto
    ${_PROTO_DIR}/ssl_gc_game_event.proto
    ${_PROTO_DIR}/ssl_gc_referee_message.proto
    ${_PROTO_DIR}/ssl_gc_rcon.proto
    ${_PROTO_DIR}/ssl_vision_detection.proto
    ${_PROTO_DIR}/ssl_vision_geometry.proto
    ${_PROTO_DIR}/ssl_vision_wrapper.proto
    ${_PROTO_DIR}/ssl_simulation_control.proto
    ${_PROTO_DIR}/ssl_simulation_error.proto
  PROTO_PATHS
    ${_PROTO_DIR}
  SIDECAR
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ssl_ros_annotations.json
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${GENERATED_ROS2_MSGS}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
