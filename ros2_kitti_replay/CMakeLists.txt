cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_replay LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(HEADER_LIST
    "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/ordered_timestamps.hpp")

set(SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/ordered_timestamps.cpp")

add_library(${PROJECT_NAME} STATIC ${SOURCE_LIST} ${HEADER_LIST})

target_include_directories(ros2_kitti_replay PUBLIC include)
ament_target_dependencies(ros2_kitti_replay rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp)
ament_package()
