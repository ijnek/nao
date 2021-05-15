cmake_minimum_required(VERSION 3.5)
project(nao_description)

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
find_package(sensor_msgs REQUIRED)

add_executable(zero_joint_state_publisher src/zero_joint_state_publisher.cpp)
ament_target_dependencies(zero_joint_state_publisher rclcpp sensor_msgs)

install(TARGETS
  zero_joint_state_publisher
  DESTINATION lib/${PROJECT_NAME})

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/meshes)
  message(FATAL_ERROR "Missing meshes directory, please follow instructions in README.md to install meshes")
endif()

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/texture)
  message(FATAL_ERROR "missing texture directory, please follow instructions in README.md to install texture")
endif()

foreach(dir urdf launch rviz meshes texture)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir}/)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
