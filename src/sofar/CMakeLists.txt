cmake_minimum_required(VERSION 3.8)
project(sofar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(odom_from_joints src/odom_from_joints.cpp)

ament_target_dependencies(odom_from_joints
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
)

install(TARGETS
  odom_from_joints
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
