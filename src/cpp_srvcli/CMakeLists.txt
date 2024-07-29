cmake_minimum_required(VERSION 3.8)
project(cpp_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_server_1 src/add_two_ints_server_1.cpp)
ament_target_dependencies(add_two_ints_server_1 rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(client_subcription_node src/client_subcription_node.cpp)
ament_target_dependencies(client_subcription_node rclcpp example_interfaces std_msgs)

add_executable(service_practice src/service_practice.cpp)
ament_target_dependencies(service_practice rclcpp turtlesim)

add_executable(service_practice_b src/service_practice_b.cpp)
ament_target_dependencies(service_practice_b rclcpp turtlesim geometry_msgs)

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

install(TARGETS
   add_two_ints_server
   add_two_ints_server_1
   add_two_ints_client
   client_subcription_node
   service_practice
   service_practice_b
   DESTINATION lib/${PROJECT_NAME})

ament_package()