cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_odom_open3d LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(Open3D REQUIRED)
find_package(open3d_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_kitti_core REQUIRED)
find_package(ros2_kitti_odom REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

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
    "include/${PROJECT_NAME}/open3d_odometry_node.hpp"
    "include/${PROJECT_NAME}/open3d_odometry_config.hpp")
set(SOURCE_LIST "src/open3d_odometry_node.cpp" "src/open3d_odometry_config.cpp")

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(${PROJECT_NAME} SHARED ${SOURCE_LIST} ${HEADER_LIST})
target_include_directories(${PROJECT_NAME}
                           PUBLIC include ${Open3D_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME}
                          Eigen3
                          Open3D
                          open3d_conversions
                          rclcpp
                          rclcpp_components
                          ros2_kitti_core
                          ros2_kitti_odom
                          tf2
                          tf2_eigen
                          tf2_geometry_msgs
                          yaml-cpp)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
rclcpp_components_register_nodes(${PROJECT_NAME}
                                 "r2k_odom_o3d::Open3DOdometryNode")
set(node_plugins
    "${node_plugins}r2k_odom_o3d::Open3DOdometryNode;\
$<TARGET_FILE:ros2_kitti_odom_open3d>\n")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  file(GENERATE
       OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index/\
       $<CONFIG>/share/ament_index/resource_index/node_plugin/${PROJECT_NAME}"
       CONTENT "${node_plugins}")

  # Bypass copyright and other flaky checks
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
