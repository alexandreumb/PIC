cmake_minimum_required(VERSION 3.8)
project(cyclone_try)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fixed_size_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera2_pub src/camera2_pub.cpp)
ament_target_dependencies(camera2_pub OpenCV rclcpp fixed_size_msgs)

add_executable(camera2_sub src/camera2_sub.cpp)
ament_target_dependencies(camera2_sub OpenCV rclcpp fixed_size_msgs)

install(TARGETS
  camera2_pub
  camera2_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
