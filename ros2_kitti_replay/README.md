ros2 launch ros2_kitti_replay replayer_bringup.launch.py
ros2 service call /resume ros2_kitti_interface/srv/Resume "{request: {replay_speed: 2.0}}"
ros2 service call /pause std_srvs/srv/Trigger


for point cloud visualization to be correct:
- replay speed must match the frame rate of RVIZ (x1.0 -> 10Hz, x2.0 -> 20Hz, etc)
