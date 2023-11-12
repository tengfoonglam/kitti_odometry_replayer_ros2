ros2 launch ros2_kitti_replay replayer_bringup.launch.py dataset_number:=3

ros2 launch ros2_kitti_replay replayer_bringup.launch.py dataset_number:=5

ros2 launch ros2_kitti_replay replayer_bringup.launch.py dataset_number:=5 start_time:=60.0 end_time:=120.0

ros2 service call /kitti_replayer/play ros2_kitti_msgs/srv/Play "{request: {replay_speed: 2.0}}"
ros2 service call /kitti_replayer/pause std_srvs/srv/Trigger
ros2 service call /kitti_replayer/step ros2_kitti_msgs/srv/Step "{request: {replay_speed: 2.0, number_steps: 50}}"
ros2 service call /kitti_replayer/set_next_play_time_range ros2_kitti_msgs/srv/SetTimeRange "{request:{start_time:{sec: 30, nanosec: 0}, end_time:{sec: 60, nanosec: 0}}}"
ros2 launch ros2_kitti_replay replayer_bringup.launch.py dataset_number:=5 odometry_package:=ros2_kitti_odom_open3d odometry_plugin:=r2k_odom_o3d::Open3DOdometryNode
