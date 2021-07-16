cmake_minimum_required(VERSION 3.5)
project(nao_state_publisher)

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
find_package(nao_sensor_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)

add_executable(nao_state_publisher src/nao_state_publisher.cpp)
ament_target_dependencies(nao_state_publisher
  rclcpp
  nao_sensor_msgs
  robot_state_publisher)

install(TARGETS nao_state_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
