# ros2_kitti_replayer

## Pre-Commit Hooks

This repository uses [pre-commit hooks](https://pre-commit.com/) to ensure that code is checked for simple issues before it is committed. This is an ***initial layer*** of checks to ensure that committed code adheres to the [ament lints](https://github.com/ament/ament_lint). **Disclaimer**: The pre-commit hooks do not run the exact same checks as *ament_lint* as some of the checks are not supported by pre-commit hooks.

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
