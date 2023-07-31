cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_replay LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_kitti_core REQUIRED)
find_package(ros2_kitti_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

set(HEADER_LIST "include/${PROJECT_NAME}/kitti_replayer_node.hpp")
set(SOURCE_LIST "src/kitti_replayer_node.cpp")

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(${PROJECT_NAME} SHARED ${HEADER_LIST} ${SOURCE_LIST})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_definitions(${PROJECT_NAME} PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME}
                          image_transport
                          nav_msgs
                          rclcpp
                          rclcpp_components
                          ros2_kitti_core
                          ros2_kitti_interface
                          std_msgs
                          std_srvs
                          tf2)
rclcpp_components_register_nodes(${PROJECT_NAME}
                                 "r2k_replay::KITTIReplayerNode")
set(node_plugins
    "${node_plugins}r2k_replay::KITTIReplayerNode;\
    $<TARGET_FILE:${PROJECT_NAME}>\n")

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(image_transport
                          nav_msgs
                          rclcpp
                          rclcpp_components
                          ros2_kitti_core
                          ros2_kitti_interface
                          std_msgs
                          std_srvs
                          tf2)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
