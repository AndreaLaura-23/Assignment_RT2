cmake_minimum_required(VERSION 3.8)
project(my_robot_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToX.action"
)

# Server
add_executable(move_to_x_server src/move_to_x_server.cpp)
ament_target_dependencies(move_to_x_server
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
)

# Client
add_executable(move_to_x_client src/move_to_x_client.cpp)
ament_target_dependencies(move_to_x_client
  rclcpp
  rclcpp_action
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(move_to_x_server "${cpp_typesupport_target}")
target_link_libraries(move_to_x_client "${cpp_typesupport_target}")

install(TARGETS
  move_to_x_server
  move_to_x_client
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_dependencies(rosidl_default_runtime)

ament_package()