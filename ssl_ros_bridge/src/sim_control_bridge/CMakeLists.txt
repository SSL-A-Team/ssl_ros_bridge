add_library(${PROJECT_NAME}_sim_control_bridge SHARED
    sim_control_bridge_node.cpp
)
target_include_directories(${PROJECT_NAME}_sim_control_bridge PRIVATE ..)
ament_target_dependencies(${PROJECT_NAME}_sim_control_bridge
  rclcpp
  rclcpp_components
  ssl_ros_bridge_msgs
  ssl_league_msgs
  ssl_league_protobufs
)
target_link_libraries(${PROJECT_NAME}_sim_control_bridge ${PROJECT_NAME}_core)

rclcpp_components_register_node(
  ${PROJECT_NAME}_sim_control_bridge
  PLUGIN "ssl_ros_bridge::sim_control_bridge::SimControlBridge"
  EXECUTABLE sim_control_bridge_node
)

install(TARGETS ${PROJECT_NAME}_sim_control_bridge DESTINATION lib)
