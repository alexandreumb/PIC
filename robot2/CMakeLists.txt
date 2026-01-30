cmake_minimum_required(VERSION 3.8)
project(robot2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fixed_size_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_hoofs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# CAMERA 1 PUB
add_executable(camera1_pub
  src/camera1_pub.cpp
)
ament_target_dependencies(camera1_pub
  rclcpp
  OpenCV
  fixed_size_msgs
  realsense2
)
target_link_libraries(camera1_pub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh ${realsense2_LIBRARY})

#CAMERA 1 SUB
add_executable(camera1_sub
  src/camera1_sub.cpp
)
ament_target_dependencies(camera1_sub
  rclcpp
  OpenCV
  fixed_size_msgs
)
target_link_libraries(camera1_sub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh)

#CAMERA 2 PUB
add_executable(camera2_pub
  src/camera2_pub.cpp
)
ament_target_dependencies(camera2_pub
  rclcpp
  OpenCV
  fixed_size_msgs
  realsense2
)
target_link_libraries(camera2_pub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh ${realsense2_LIBRARY})

#CAMERA 2 SUB
add_executable(camera2_sub
  src/camera2_sub.cpp
)
ament_target_dependencies(camera2_sub
  rclcpp
  OpenCV
  fixed_size_msgs
)
target_link_libraries(camera2_sub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh)

#ROBOT
add_library(robot2 
  SHARED
  hardware/robot.cpp
)
# Include headers
target_include_directories(robot2 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/robot2>
)
ament_target_dependencies(robot2
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  controller_manager
  pluginlib
  serial
)
pluginlib_export_plugin_description_file(hardware_interface robot2.xml)

install(TARGETS
  camera1_pub camera1_sub camera2_pub camera2_sub
  RUNTIME DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY hardware/include/
  DESTINATION include/robot2
)

install(DIRECTORY description/launch description/urdf description/ros2_control_description
  DESTINATION share/robot2
)

install(DIRECTORY bringup/config
  DESTINATION share/robot2
)

install(TARGETS robot2
  EXPORT export_robot2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
