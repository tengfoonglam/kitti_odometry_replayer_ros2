ros2 launch ros2_kitti_replay replayer_bringup.launch.py
ros2 service call /kitti_replayer/play ros2_kitti_interface/srv/Play "{request: {replay_speed: 2.0}}"
ros2 service call /kitti_replayer/pause std_srvs/srv/Trigger
ros2 service call /kitti_replayer/step ros2_kitti_interface/srv/Step "{request: {replay_speed: 2.0, number_steps: 50}}"
ros2 service call /kitti_replayer/set_time_range ros2_kitti_interface/srv/SetTimeRange "{request:{start_time:{sec: 30, nanosec: 0}, end_time:{sec: 60, nanosec: 0}}}"

for point cloud visualization to be correct:
- replay speed must match the frame rate of RVIZ (x1.0 -> 10Hz, x2.0 -> 20Hz, etc)


for optimal performance:
- place dataset on a SSD card
- if 30s delay between frames, try another USB Port
