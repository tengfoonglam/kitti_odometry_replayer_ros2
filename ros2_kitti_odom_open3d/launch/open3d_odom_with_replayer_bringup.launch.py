from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    dataset_number = LaunchConfiguration("dataset_number", default="00")
    odometry_config_name = LaunchConfiguration(
        "odometry_config_name", default="default"
    )
    odometry_config_path = PathJoinSubstitution(
        [
            PythonExpression(
                [
                    '"" if len("',
                    odometry_config_name,
                    '") == 0 else "',
                    get_package_share_directory("ros2_kitti_odom_open3d"),
                    "/config/",
                    odometry_config_name,
                    '.yml"',
                ]
            ),
        ]
    )
    start_time = LaunchConfiguration("start_time", default="0.0")
    end_time = LaunchConfiguration("end_time", default="0.0")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_number",
                default_value="00",
                description="Dataset number to load. Omit any leading zeros.",
            ),
            DeclareLaunchArgument(
                "odometry_config_name",
                default_value="default",
                description="Name of the config file used for the odometry component "
                "(exclude .yml). Leave empty to launch default values",
            ),
            DeclareLaunchArgument(
                "start_time",
                default_value="0.0",
                description="Start time of the replayer. Set to zero to play the "
                "entire duration",
            ),
            DeclareLaunchArgument(
                "end_time",
                default_value="0.0",
                description="End time of the replayer. Set to zero to play the "
                "entire duration",
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("ros2_kitti_replay"),
                        "/launch/replayer_bringup.launch.py",
                    ]
                ),
                launch_arguments={
                    "dataset_number": dataset_number,
                    "odometry_package": "ros2_kitti_odom_open3d",
                    "odometry_plugin": "r2k_odom_o3d::Open3DOdometryNode",
                    "odometry_config_path": odometry_config_path,
                    "vehicle_sensor_link": "lidar",
                    "enable_point_cloud": "true",
                    "enable_gray_images": "false",
                    "enable_colour_images": "false",
                    "start_time": start_time,
                    "end_time": end_time,
                }.items(),
            ),
        ]
    )
