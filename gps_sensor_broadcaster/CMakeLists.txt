cmake_minimum_required(VERSION 3.5)
project(gps_sensor_broadcaster)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gps_sensor_broadcaster.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  sensor_msgs
  realtime_tools
)

# export plugin XML
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/gps_sensor_broadcaster
)

install(FILES
  gps_sensor_broadcaster_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  controller_interface
  gps_sensor_broadcaster_plugins.xml
)

ament_package()
