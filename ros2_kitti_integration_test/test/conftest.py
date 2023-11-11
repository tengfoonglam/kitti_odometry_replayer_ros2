import launch_pytest
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

from pathlib import Path

TEST_DATA_PATH = Path().home() / "ros2_kitti_test_data"
TEST_DATASET_NUMBER = "0"
DEFAULT_START_TIME = "0"
DEFAULT_END_TIME = "0"


@launch_pytest.fixture
def launch_replayer_with_full_test_data() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("ros2_kitti_replay"),
                        "/launch/replayer_bringup.launch.py",
                    ]
                ),
                launch_arguments={
                    "dataset_number": TEST_DATASET_NUMBER,
                    "vehicle_sensor_link": "lidar",
                    "enable_point_cloud": "true",
                    "enable_gray_images": "false",
                    "enable_colour_images": "false",
                    "start_time": DEFAULT_START_TIME,
                    "end_time": DEFAULT_END_TIME,
                }.items(),
            ),
        ]
    )
