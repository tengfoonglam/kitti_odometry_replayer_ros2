# ros2_kitti

[![CI](https://github.com/tengfoonglam/ros2_kitti/actions/workflows/main.yml/badge.svg)](https://github.com/tengfoonglam/ros2_kitti/actions/workflows/main.yml)

## Setup

Clone this repository in the `src` folder of your ROS2 workspace (e.g. ros2_ws folder)

#### Without Virtual Environment

1. Install dependencies using *rosdep*: `rosdep install -i --from-path src --ignore-src --rosdistro $ROS_DISTRO -y`
   * Note: Specify to not use *sudo* when which will install pip packages to OS Python environment
2. Build package: `colcon build --packages-up-to ros2_kitti`

#### With Virtual Environment

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
