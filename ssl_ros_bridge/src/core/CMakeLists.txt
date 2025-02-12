add_library(${PROJECT_NAME}_core SHARED
    get_ip_addresses.cpp
    message_conversion.cpp
    multicast_receiver.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC .)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  ssl_league_msgs
  ssl_league_protobufs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_core
  Boost::boost
  protobuf::libprotobuf
)
install(TARGETS ${PROJECT_NAME}_core DESTINATION lib)
