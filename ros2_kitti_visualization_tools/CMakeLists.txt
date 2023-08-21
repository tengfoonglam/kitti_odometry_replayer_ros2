cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_visualization_tools LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

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

set(HEADER_LIST "include/${PROJECT_NAME}/path_publisher.hpp")

set(SOURCE_LIST "src/path_publisher.cpp")

add_library(${PROJECT_NAME} STATIC ${SOURCE_LIST} ${HEADER_LIST})

target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
                          geometry_msgs
                          rclcpp
                          visualization_msgs)

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
ament_export_dependencies(geometry_msgs rclcpp visualization_msgs)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/)

ament_package()
