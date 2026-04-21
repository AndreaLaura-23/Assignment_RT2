cmake_minimum_required(VERSION 3.8)
project(assignment1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)


# Messages and Actions

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UiGoal.msg"
  "action/NavigateToPose.action"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)


# Action Server Component

add_library(action_server SHARED
  src/action_server.cpp
)

ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(action_server
  ${cpp_typesupport_target}
)

rclcpp_components_register_nodes(action_server
  "assignment1::ActionServer"
)


# Action Client Component

add_library(action_client SHARED
  src/action_client.cpp
)

ament_target_dependencies(action_client
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
)

target_link_libraries(action_client
  ${cpp_typesupport_target}
)

rclcpp_components_register_nodes(action_client
  "assignment1::ActionClient"
)


# User Interface Node

add_executable(user_interface
  src/user_interface.cpp
)

ament_target_dependencies(user_interface
  rclcpp
  std_msgs
)

target_link_libraries(user_interface
  ${cpp_typesupport_target}
)


# Install

install(TARGETS
  action_server
  action_client
  user_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# Testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()