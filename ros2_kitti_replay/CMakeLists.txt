cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_replay LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall
                      -Wextra
                      -Wshadow
                      -Wnon-virtual-dtor
                      -pedantic)
endif()

set(HEADER_LIST
    "include/${PROJECT_NAME}/timestamps.hpp"
    "include/${PROJECT_NAME}/data_loader_base.hpp"
    "include/${PROJECT_NAME}/data_loader.hpp"
    "include/${PROJECT_NAME}/clock_data_loader.hpp"
    "include/${PROJECT_NAME}/play_data_callback_base.hpp"
    "include/${PROJECT_NAME}/point_cloud_utils.hpp"
    "include/${PROJECT_NAME}/data_replayer.hpp"
    "include/${PROJECT_NAME}/ros_publisher_play_data_callback.hpp")

set(SOURCE_LIST
    "src/timestamps.cpp"
    "src/data_loader_base.cpp"
    "src/data_replayer.cpp"
    "src/point_cloud_utils.cpp")

add_library(${PROJECT_NAME} STATIC ${SOURCE_LIST} ${HEADER_LIST})

target_include_directories(ros2_kitti_replay PUBLIC include)
ament_target_dependencies(ros2_kitti_replay
                          rclcpp
                          sensor_msgs
                          pcl_conversions)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Bypass copyright and uncrustify checks
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

add_subdirectory(apps)

ament_export_include_directories(include)
ament_export_libraries(ros2_kitti_replay)
ament_export_dependencies(rclcpp sensor_msgs)

install(TARGETS convert_kitti_point_cloud DESTINATION lib/${PROJECT_NAME})

ament_package()
