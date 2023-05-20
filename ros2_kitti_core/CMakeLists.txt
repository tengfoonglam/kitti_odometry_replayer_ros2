cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_core LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(ros2_kitti_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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
    "include/${PROJECT_NAME}/timestamp_utils.hpp"
    "include/${PROJECT_NAME}/data_loader_base.hpp"
    "include/${PROJECT_NAME}/data_loader.hpp"
    "include/${PROJECT_NAME}/clock_data_loader.hpp"
    "include/${PROJECT_NAME}/play_data_interface_base.hpp"
    "include/${PROJECT_NAME}/point_cloud_utils.hpp"
    "include/${PROJECT_NAME}/point_cloud_data_loader.hpp"
    "include/${PROJECT_NAME}/data_replayer.hpp"
    "include/${PROJECT_NAME}/load_and_play_data_interface.hpp"
    "include/${PROJECT_NAME}/pose_utils.hpp"
    "include/${PROJECT_NAME}/pose_data_loader.hpp")

set(SOURCE_LIST
    "src/timestamp_utils.cpp"
    "src/data_loader_base.cpp"
    "src/clock_data_loader.cpp"
    "src/data_replayer.cpp"
    "src/point_cloud_utils.cpp"
    "src/point_cloud_data_loader.cpp"
    "src/pose_utils.cpp"
    "src/pose_data_loader.cpp")

add_library(${PROJECT_NAME} STATIC ${SOURCE_LIST} ${HEADER_LIST})

target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
                          geometry_msgs
                          PCL
                          pcl_ros
                          pcl_conversions
                          rclcpp
                          rosgraph_msgs
                          ros2_kitti_interface
                          sensor_msgs
                          std_srvs
                          tf2
                          tf2_geometry_msgs)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(kitti_replayer_component SHARED
            "src/kitti_replayer_node.cpp"
            "include/${PROJECT_NAME}/kitti_replayer_node.hpp")
target_include_directories(kitti_replayer_component
                           PUBLIC "include/${PROJECT_NAME}/\
                           kitti_replayer_node.hpp")
target_link_libraries(kitti_replayer_component ${PROJECT_NAME})
target_compile_definitions(kitti_replayer_component
                           PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(kitti_replayer_component
                          nav_msgs
                          rclcpp
                          rclcpp_components
                          ros2_kitti_interface
                          std_msgs
                          tf2)
rclcpp_components_register_nodes(kitti_replayer_component
                                 "r2k_core::KITTIReplayerNode")
set(node_plugins
    "${node_plugins}r2k_core::KITTIReplayerNode;\
    $<TARGET_FILE:kitti_replayer_component>\n")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  file(GENERATE
       OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/\
       $<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
       CONTENT "${node_plugins}")

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
ament_export_libraries(ros2_kitti_core)
ament_export_dependencies(rclcpp
                          rclcpp_components
                          geometry_msgs
                          PCL
                          pcl_ros
                          pcl_conversions
                          rosgraph_msgs
                          ros2_kitti_interface
                          sensor_msgs
                          std_srvs
                          tf2
                          tf2_geometry_msgs)

install(TARGETS ${PROJECT_NAME} kitti_replayer_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS convert_kitti_point_cloud DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
