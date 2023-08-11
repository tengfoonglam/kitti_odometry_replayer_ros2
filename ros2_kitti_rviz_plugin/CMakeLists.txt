cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_rviz_plugin LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(ros2_kitti_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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

# This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

set(HEADER_LIST "include/${PROJECT_NAME}/replay_panel.hpp")

set(SOURCE_LIST "src/replay_panel.cpp")

# An RVIZ plugin must be a shared library
add_library(${PROJECT_NAME} SHARED ${SOURCE_LIST} ${HEADER_LIST})
ament_target_dependencies(${PROJECT_NAME}
                          rclcpp
                          ros2_kitti_msgs
                          rviz_common
                          rviz_rendering
                          std_msgs
                          std_srvs)
target_include_directories(${PROJECT_NAME}
                           PUBLIC include ${Qt5Widgets_INCLUDE_DIRS})

# Here we export the plugins and meshes so they can be found by RViz at runtime.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

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
ament_export_dependencies(rclcpp
                          ros2_kitti_msgs
                          rviz_common
                          rviz_rendering)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_package()
