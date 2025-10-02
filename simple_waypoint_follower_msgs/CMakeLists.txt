cmake_minimum_required(VERSION 3.8)
project(simple_waypoint_follower_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VariableWaypointRadius.msg"
  "msg/Function.msg"
  "msg/Waypoint.msg"
  "msg/Waypoints.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp"
)

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package()
