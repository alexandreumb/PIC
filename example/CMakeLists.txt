#example
cmake_minimum_required(VERSION 3.8)
project(example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(serial REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fixed_size_msgs REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_hoofs REQUIRED)
find_package(realsense2 REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ----------------------------
# Camera nodes
# ----------------------------
add_executable(camera1_pub src/camera1_pub.cpp)
ament_target_dependencies(camera1_pub
  rclcpp OpenCV fixed_size_msgs realsense2
)
target_link_libraries(camera1_pub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh ${realsense2_LIBRARY})

# Install Python modules
ament_python_install_package(robot_control)
install(PROGRAMS
  robot_control/control_validator.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(camera1_sub src/camera1_sub.cpp)
ament_target_dependencies(camera1_sub rclcpp OpenCV fixed_size_msgs)
target_link_libraries(camera1_sub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh)
# add to CMakeLists.txt temporarily
# target_compile_options(camera1_sub PRIVATE -fsanitize=address)
# target_link_options(camera1_sub PRIVATE -fsanitize=address)

add_executable(camera_fastrtps_pub src/camera_fastrtps_pub.cpp)
ament_target_dependencies(camera_fastrtps_pub rclcpp OpenCV fixed_size_msgs realsense2)
target_link_libraries(camera_fastrtps_pub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh ${realsense2_LIBRARY})

add_executable(camera_fastrtps_sub src/camera_fastrtps_sub.cpp)
ament_target_dependencies(camera_fastrtps_sub rclcpp OpenCV fixed_size_msgs)
target_link_libraries(camera_fastrtps_sub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh)

# ── CycloneDDS camera publisher ───────────────────────────────────────────────
add_executable(camera_publisher_cyclone
  src/camera_publisher_cyclone.cpp
)
ament_target_dependencies(camera_publisher_cyclone
  rclcpp
  fixed_size_msgs
  OpenCV
)

# ── CycloneDDS camera subscriber ──────────────────────────────────────────────
add_executable(camera_subscriber_cyclone
  src/camera_subscriber_cyclone.cpp
)
ament_target_dependencies(camera_subscriber_cyclone
  rclcpp
  fixed_size_msgs
  OpenCV
)

# ── install all executables ───────────────────────────────────────────────────
install(TARGETS
  camera_publisher_cyclone
  camera_subscriber_cyclone
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------
# GPS hardware plugin (for simulation)
# ----------------------------
add_library(example SHARED 
    hardware/example.cpp
    hardware/robot.cpp)
target_include_directories(example PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/example>
)
ament_target_dependencies(example
  rclcpp rclcpp_lifecycle hardware_interface controller_manager pluginlib serial
)
pluginlib_export_plugin_description_file(hardware_interface example.xml)

# ----------------------------
# Install targets
# ----------------------------
install(TARGETS
  camera1_pub camera1_sub camera_fastrtps_pub camera_fastrtps_sub
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY hardware/include/
  DESTINATION include/example
)

install(DIRECTORY description/urdf description/ros2_control_description description/launch
  DESTINATION share/example
)

install(DIRECTORY bringup/config
  DESTINATION share/example
)

install(TARGETS example
  EXPORT export_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ----------------------------
# Testing / linting
# ----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
