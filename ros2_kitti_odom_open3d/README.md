rosdep install -i --from-path src/ros2_kitti/ros2_kitti_odom_open3d/ --rosdistro $ROS_DISTRO -y

ros2 launch ros2_kitti_odom_open3d open3d_odom_with_replayer_bringup.launch.py dataset_number:=5

ros2 launch ros2_kitti_odom_open3d open3d_odom_with_replayer_bringup.launch.py dataset_number:=5 start_time:=60.0 end_time:=120.0
