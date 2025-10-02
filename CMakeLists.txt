cmake_minimum_required(VERSION 3.8)
project(simple_waypoint_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_wf
  src/simple_waypoint_follower.cpp
)

rosidl_generate_interfaces(simple_waypoint_follower_msgs
  "msg/function/VariableWaypointRadius.msg"
  "msg/Function.msg"
  "msg/Waypoint.msg"
  "msg/Waypoints.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target "simple_waypoint_follower_msgs" "rosidl_typesupport_cpp"
)

ament_export_dependencies(rosidl_default_runtime)


install(TARGETS simple_wf DESTINATION lib/${PROJECT_NAME} )
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
