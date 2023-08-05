cmake_minimum_required(VERSION 3.5)
project(cpp07_exercise)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(base_interfaces_demo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(exe01_pub_sub src/exe01_pub_sub.cpp)
add_executable(exe02_server src/exe02_server.cpp)
add_executable(exe02_client src/exe02_client.cpp)
add_executable(exe03_action_server src/exe03_action_server.cpp)
add_executable(exe03_action_client src/exe03_action_client.cpp)
add_executable(exe04_param src/exe04_param.cpp)

target_include_directories(exe01_pub_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  exe01_pub_sub
  "rclcpp"
  "turtlesim"
  "base_interfaces_demo"
  "geometry_msgs"
  "rclcpp_action"
)

ament_target_dependencies(
  exe02_server
  "rclcpp"
  "turtlesim"
  "base_interfaces_demo"
  "geometry_msgs"
  "rclcpp_action"
)

ament_target_dependencies(
  exe02_client
  "rclcpp"
  "turtlesim"
  "base_interfaces_demo"
  "geometry_msgs"
  "rclcpp_action"
)

ament_target_dependencies(
  exe03_action_server
  "rclcpp"
  "turtlesim"
  "base_interfaces_demo"
  "geometry_msgs"
  "rclcpp_action"
)

ament_target_dependencies(
  exe03_action_client
  "rclcpp"
  "turtlesim"
  "base_interfaces_demo"
  "geometry_msgs"
  "rclcpp_action"
)

ament_target_dependencies(
  exe04_param
  "rclcpp"
  "turtlesim"
  "base_interfaces_demo"
  "geometry_msgs"
  "rclcpp_action"
)

install(TARGETS 
  exe01_pub_sub
  exe02_server
  exe02_client
  exe03_action_server
  exe03_action_client
  exe04_param
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
