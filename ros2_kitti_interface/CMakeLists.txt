cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_interface)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
                           "msg/PlayRequest.msg"
                           "msg/ReplayerState.msg"
                           "msg/ResumeRequest.msg"
                           "msg/StepRequest.msg"
                           DEPENDENCIES
                           std_msgs
                           ADD_LINTER_TESTS)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
