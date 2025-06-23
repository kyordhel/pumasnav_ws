cmake_minimum_required(VERSION 3.14)
project(navig_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  "srv/GetGoalPoint.srv"
  "srv/GetInflatedMap.srv"
  "srv/GetBoundaryPoints.srv"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs graph_msgs
)

ament_package()
