add_executable(${PROJECT_NAME}_log2bag
  log2bag.cpp
  log_reader.cpp
)
set_target_properties(${PROJECT_NAME}_log2bag PROPERTIES OUTPUT_NAME log2bag)
target_include_directories(${PROJECT_NAME}_log2bag PRIVATE ..)
target_compile_features(${PROJECT_NAME}_log2bag PUBLIC cxx_std_20)
ament_target_dependencies(${PROJECT_NAME}_log2bag
  rclcpp
  rosbag2_cpp
  ssl_league_msgs
  ssl_league_protobufs
)
target_link_libraries(${PROJECT_NAME}_log2bag ${PROJECT_NAME}_core)

install(TARGETS ${PROJECT_NAME}_log2bag DESTINATION lib/${PROJECT_NAME})
