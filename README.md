# KITTI Odometry Replayer ROS 2

[![CI](https://github.com/tengfoonglam/ros2_kitti/actions/workflows/main.yml/badge.svg)](https://github.com/tengfoonglam/ros2_kitti/actions/workflows/main.yml)

[INSERT IMAGE]

**KITTI Odometry Replayer ROS 2** is a ROS 2 package that allows a user to replay data from the [KITTI Odometry Dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php). Currently only supports the **Humble** distribution.

Key features of the package:
1. Replayer that replays all the data provided by the KITTI Odometry Dataset
    * Timestamps
    * Ground truth poses
    * Lidar point cloud
    * Left/right grayscale images
    * Left/right colour images
1. Additional replayer functionality
    * Start/pause
    * Replay at a specified speed
    * Replay only a segment of a sequence
    * Replayer can be made a [composible node](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html) which allows for more efficient transport of published data
1. URDF generator that creates the URDF of all the vehicle setup in the dataset
1. Composible odometry nodes that uses the data replayer as a data source:
    * Odometry using [Open3D](http://www.open3d.org/)'s point cloud resigration implementation
    * [KISS ICP](https://github.com/PRBonn/kiss-icp)
1. Visualization tools
    * RVIZ plugin that allows the control of the replayer via a GUI
    * 'Ghost car' driving on the ground truth path when running the replayer with an odometry node

**Note**: The repository is still a work in progress hence so if there are any problems please file an issue and I will try and address it. Thank you for your patience.

## Packages in this Repository

 * **ros2_kitti_core**: Core functionality of the loading data from the dataset folders
 * **ros2_kitti_description**: Generate URDF of the vehicle in the sequences based on the provided calibration data
 * **ros2_kitti_msgs**: Service/message definitions by the replayer
 * **ros2_kitti_odom**: Base class for the odometry node
 * **ros2_kitti_odom_kiss_icp**: Odometry node implementation using KISS ICP
 * **ros2_kitti_odom_open3d**: Odometry node implementation using Open3D point cloud registration
 * **ros2_kitti_replay**: Replayer node
 * **ros2_kitti_rviz_plugin**: RVIZ plugin that interacts with the replayer node

## Setup

Clone this repository in the `src` folder of your ROS2 workspace (e.g. ros2_ws folder)

#### Without Virtual Environment

1. Install dependencies using *rosdep*: `rosdep install --from-path src --ignore-src --rosdistro $ROS_DISTRO -y`
2. Build package: `colcon build --packages-up-to ros2_kitti`

#### With Virtual Environment

**Note**: This is not required unless you want to run the scripts in **ros2_kitti_description**, which is not really required as well as the URDF for each sequence has been generated and committed into the repository for the convenience of the user.

1. In ROS2 Workspace, create virtual environment: `virtualenv -p python3 --system-site-packages --symlinks .venv`
   * Note: *rosdep* installs some dependencies using *apt* so we need the `--system-site-packages` flag to access them
   * For more information see this [issue](https://github.com/ros2/ros2/issues/1094)
2. Ensure colcon does not try to build the new virtual environment (Source [here](https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html)): `touch .venv/COLCON_IGNORE`
3. Activate virtual environment: `source .venv/bin/activate`
4. Install dependencies using *rosdep*: `rosdep install -i --from-path src --ignore-src --rosdistro $ROS_DISTRO -y --as-root pip:no`
   * Note: Specify to not use *sudo* when which will install pip packages to OS Python environment
5. Unfortunately, some dependencies cannot be satisfied using rosdep and needs to be resolved directly using pip
   * With .venv activated, in your workspace directory run `pip install -r src/ros2_kitti/non_rosdep_requirements.txt`
   * Note: Ignore warnings, see comments in `non_rosdep_requirements.txt` for more details
6. Build package: `colcon build --packages-up-to ros2_kitti`
7. Source the workspace after building `source ./install/setup.sh`

### Downloading the Dataset

In your Home directory, create a folder named `kitti_dataset`. Download the dataset, and extract the data into it. In the event that you want the files somewhere else, edit the variable `DATASET_PATH` in `ros2_kitti_replay/launch/replayer_bringup.launch.py`.

The `kitti_dataset` folder should look something like this.

```
kitti_dataset
├── data_odometry_calib
│   └── dataset
│       └── sequences
├── data_odometry_color
│   └── dataset
│       └── sequences
├── data_odometry_gray
│   └── dataset
│       └── sequences
├── data_odometry_poses
│   └── dataset
│       └── poses
└── data_odometry_velodyne
    └── dataset
        └── sequences
```

In the event that you do not want to process/download the point cloud, grayscale images or colour images, change the value of the variables `POINT_CLOUD_FOLDER_AVAILABLE`, `GRAY_IMAGES_FOLDER_AVAILABLE`, `COLOUR_IMAGES_FOLDER_AVAILABLE` to in `ros2_kitti_replay/launch/replayer_bringup.launch.py` to reflect this.

## Quick Start

##### Replay a dataset sequence (e.g. Sequence 1). Press play using the RVIZ GUI.

`ros2 launch ros2_kitti_replay replayer_bringup.launch.py dataset_number:=1`

##### Run a dataset sequence in a specified time range

`ros2 launch ros2_kitti_replay replayer_bringup.launch.py dataset_number:=5 start_time:=60.0 end_time:=120.0`

##### Run Open3D ICP Odometry over a dataset sequence

`ros2 launch ros2_kitti_odom_open3d open3d_odom_with_replayer_bringup.launch.py dataset_number:=1`

##### Run KISS ICP Odometry over a dataset sequence

`ros2 launch ros2_kitti_odom_kiss_icp kiss_icp_odom_with_replayer_bringup.launch.py dataset_number:=1`

## Terminal Commands while the Replayer is Running

##### Play

`ros2 service call /kitti_replayer/play ros2_kitti_msgs/srv/Play "{request: {replay_speed: 2.0}}" `

##### Pause

`ros2 service call /kitti_replayer/pause std_srvs/srv/Trigger`

##### Step forward a specified number of frames

`ros2 service call /kitti_replayer/step ros2_kitti_msgs/srv/Step "{request: {replay_speed: 2.0, number_steps: 50}}" `

##### Specify time range on the next play command

`ros2 service call /kitti_replayer/set_next_play_time_range ros2_kitti_msgs/srv/SetTimeRange "{request:{start_time:{sec: 30, nanosec: 0}, end_time:{sec: 60, nanosec: 0}}}" `

## Pre-Commit Hooks

This repository uses [pre-commit hooks](https://pre-commit.com/) to ensure that code is checked for simple issues before it is committed. This is an ***initial layer*** of checks to ensure that committed code adheres to the [ament linters](https://github.com/ament/ament_lint). **Disclaimer**: The pre-commit hooks do not run the exact same checks as *ament_lint* as some of the checks are not supported by pre-commit hooks.

#### Pre-Commit Hooks Setup

* Install pre-commit: `sudo apt install pre-commit`
* **Note**: xmllint requires [Docker](https://www.docker.com/) usages without admin privileges
  * `sudo apt install docker.io`
  * `sudo groupadd docker`
  * `sudo gpasswd -a $USER docker`
  * `newgrp docker`

#### Running the hooks

* Run hooks on every commit: `pre-commit install`
* Running the hooks manually: `pre-commit run --all`

## Known Issues

 * For point cloud visualization to be correct, replay speed must match the frame rate of RVIZ (x1.0 -> 10Hz, x2.0 -> 20Hz, etc) which can be changed in the left Display panel
 * For optimal performance, the dataset folders should be located on a SSD drive. Hard disk drives (HDD) may experience lag during the replaying.

## TODOs

- [ ] Integration tests using [launch testing](https://github.com/ros2/launch/tree/rolling/launch_testing) to test core functionality of replayer
- [ ] Report unit test coverage of **ros2_kitti_core** in CI
- [ ] Update changelog
- [ ] More documentation of
   * Transformations in the vehicle model
   * Software architecture
- [ ] Odometry node that uses visual odometry (e.g. using [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3))
