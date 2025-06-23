cmake_minimum_required(VERSION 3.14)
project(path_planner)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(navig_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(actionlib_msgs REQUIRED)

add_executable(path_planner_node
  src/node.cpp
  src/path_planner.cpp
  src/main.cpp
)

target_include_directories(path_planner_node
  PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

ament_target_dependencies(path_planner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  navig_msgs
  # actionlib_msgs
  # tf2
  # tf2_ros
)

install(TARGETS
  path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament_package()
