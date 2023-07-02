cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_odom LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
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

set(HEADER_LIST "include/${PROJECT_NAME}/odometry_node_base.hpp")

set(SOURCE_LIST "src/odometry_node_base.cpp")

add_library(${PROJECT_NAME} STATIC ${SOURCE_LIST} ${HEADER_LIST})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
                          nav_msgs
                          rclcpp
                          ros2_kitti_interface
                          sensor_msgs
                          std_msgs
                          std_srvs
                          tf2
                          tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

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
ament_export_dependencies(nav_msgs
                          rclcpp
                          ros2_kitti_interface
                          sensor_msgs
                          std_msgs
                          std_srvs
                          tf2
                          tf2_geometry_msgs)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/)

ament_package()
