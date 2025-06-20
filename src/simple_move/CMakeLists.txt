cmake_minimum_required(VERSION 3.14)
project(simple_move)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(actionlib_msgs REQUIRED)

add_executable(simple_move_node
  src/simple_move_node.cpp
)

ament_target_dependencies(simple_move_node
  std_msgs
  nav_msgs
  geometry_msgs
  actionlib_msgs
  tf2
  tf2_ros
)

install(TARGETS
  simple_move_node
)

ament_package()
