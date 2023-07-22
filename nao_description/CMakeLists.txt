cmake_minimum_required(VERSION 3.5)
project(nao_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf/)

# Use IGNORE_MESHES_AND_TEXTURE_DIRS flag to ignore these directories when using Github Actions CI
if("$ENV{IGNORE_MESHES_AND_TEXTURE_DIRS}" STREQUAL "")
  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/meshes)
    message(FATAL_ERROR "Manual installation of meshes is required. Please run ./src/nao/nao_description/install.sh in your workspace, and follow installation instructions.")
  endif()

  if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/texture)
    message(FATAL_ERROR "Manual installation of textures is required. Please run ./src/nao/nao_description/install.sh in your workspace, and follow installation instructions.")
  endif()

  foreach(dir meshes texture)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir}/)
  endforeach()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
