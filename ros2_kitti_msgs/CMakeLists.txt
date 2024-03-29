cmake_minimum_required(VERSION 3.5)

project(ros2_kitti_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
                           "msg/ReplayerState.msg"
                           "msg/PlayRequest.msg"
                           "msg/StepRequest.msg"
                           "msg/TriggerResponse.msg"
                           "srv/Play.srv"
                           "srv/Step.srv"
                           DEPENDENCIES
                           geometry_msgs
                           std_msgs
                           ADD_LINTER_TESTS)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
