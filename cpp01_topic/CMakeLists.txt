cmake_minimum_required(VERSION 3.5)
project(cpp01_topic)

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
find_package(std_msgs REQUIRED)
find_package(base_interfaces_demo REQUIRED)

add_executable(demo01_talker_str src/demo01_talker_str.cpp)
add_executable(demo02_listener_str src/demo02_listener_str.cpp)
add_executable(demo03_talker_stu src/demo03_talker_stu.cpp)
add_executable(demo04_listener_stu src/demo04_listener_stu.cpp)
add_executable(demo_pub src/demo_pub.cpp)
add_executable(demo_sub src/demo_sub.cpp)
add_executable(demo_pub_person src/demo_pub_person.cpp)
add_executable(demo_sub_person src/demo_sub_person.cpp)

target_include_directories(demo01_talker_str PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  demo01_talker_str
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
)
ament_target_dependencies(
  demo02_listener_str
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
)
ament_target_dependencies(
  demo03_talker_stu
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
)
ament_target_dependencies(
  demo04_listener_stu
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
)
ament_target_dependencies(
  demo_pub
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
)
ament_target_dependencies(
  demo_sub
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
)
ament_target_dependencies(
  demo_pub_person
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
)
ament_target_dependencies(
  demo_sub_person
  "rclcpp"
  "std_msgs"
  "base_interfaces_demo"
)

install(TARGETS 
  demo01_talker_str
  demo02_listener_str
  demo03_talker_stu
  demo04_listener_stu
  demo_pub
  demo_sub
  demo_pub_person
  demo_sub_person
  DESTINATION lib/${PROJECT_NAME})

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
