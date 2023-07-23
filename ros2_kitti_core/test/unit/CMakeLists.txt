ament_find_gtest()
add_library(ros2_kitti_core_test INTERFACE)
target_include_directories(ros2_kitti_core_test INTERFACE include)
target_link_libraries(ros2_kitti_core_test
                      INTERFACE ${GTEST_LIBRARIES}
                                opencv_core
                                opencv_imgproc
                                opencv_imgcodecs)

ament_add_gtest(timestamp_utils_test timestamp_utils_test.cpp)
target_link_libraries(timestamp_utils_test ros2_kitti_core_test ${PROJECT_NAME})

ament_add_gtest(base_data_loader_test data_loader_base_test.cpp)
target_link_libraries(base_data_loader_test ${PROJECT_NAME})

ament_add_gtest(data_replayer_static_test data_replayer_static_test.cpp)
target_link_libraries(data_replayer_static_test
                      ros2_kitti_core_test
                      ${PROJECT_NAME})

ament_add_gtest(data_replayer_test data_replayer_test.cpp)
target_link_libraries(data_replayer_test ros2_kitti_core_test ${PROJECT_NAME})

ament_add_gtest(point_cloud_utils_test point_cloud_utils_test.cpp)
ament_target_dependencies(point_cloud_utils_test pcl_conversions)
target_link_libraries(point_cloud_utils_test
                      ros2_kitti_core_test
                      ${PROJECT_NAME})

ament_add_gtest(point_cloud_data_loader_test point_cloud_data_loader_test.cpp)
ament_target_dependencies(point_cloud_data_loader_test pcl_conversions)
target_link_libraries(point_cloud_data_loader_test
                      ros2_kitti_core_test
                      ${PROJECT_NAME})

ament_add_gtest(pose_utils_test pose_utils_test.cpp)
ament_target_dependencies(pose_utils_test tf2)
target_link_libraries(pose_utils_test ros2_kitti_core_test ${PROJECT_NAME})

ament_add_gtest(pose_data_loader_test pose_data_loader_test.cpp)
ament_target_dependencies(pose_data_loader_test tf2)
target_link_libraries(pose_data_loader_test
                      ros2_kitti_core_test
                      ${PROJECT_NAME})

ament_add_gtest(load_and_play_data_interface_test
                load_and_play_data_interface_test.cpp)
target_link_libraries(load_and_play_data_interface_test
                      ros2_kitti_core_test
                      ${PROJECT_NAME})

ament_add_gtest(timer_test timer_test.cpp)
target_link_libraries(timer_test ${PROJECT_NAME})

ament_add_gtest(image_utils_test image_utils_test.cpp)
target_link_libraries(image_utils_test
                      ros2_kitti_core_test
                      opencv_core
                      ${PROJECT_NAME})
