# ros2_kitti

## Setup

#### Without Virtual Environment

1. Install dependencies using *rosdep*: `rosdep install -i --from-path src --ignore-src --rosdistro $ROS_DISTRO -y`
   * Note: Specify to not use *sudo* when which will install pip packages to OS Python environment
2. Build package: `colcon build --packages-up-to ros2_kitti`

#### With Virtual Environment

1. In ROS2 Workspace, create virtual environment: `virtualenv -p python3 --system-site-packages .venv`
   * Note: *rosdep* installs some dependencies using *apt* so we need the `--system-site-packages` flag to access them
2. Ensure colcon does not try to build the new virtual environment (Source [here](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html)): `touch .venv/COLCON_IGNORE`
3. Activate virtual environment: `source .venv/bin/activate`
4. Install dependencies using *rosdep*: `rosdep install -i --from-path src --ignore-src --rosdistro $ROS_DISTRO -y --as-root pip:no`
   * Note: Specify to not use *sudo* when which will install pip packages to OS Python environment
5. Build package: `colcon build --packages-up-to ros2_kitti`

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
