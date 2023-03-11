ament_add_gtest(timestamps_test timestamps_test.cpp)
target_link_libraries(timestamps_test ${PROJECT_NAME})

ament_add_gtest(base_data_loader_test data_loader_base_test.cpp)
target_link_libraries(base_data_loader_test ${PROJECT_NAME})

ament_add_gtest(data_replayer_static_test data_replayer_static_test.cpp)
target_link_libraries(data_replayer_static_test ${PROJECT_NAME})
