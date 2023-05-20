import os

# from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    dataset_number = LaunchConfiguration("dataset_number", default="00")
    dataset_path = LaunchConfiguration(
        "dataset_path", default="/media/ltf/LTFUbuntuSSD/kitti_dataset"
    )

    dataset_number_padded = PythonExpression(['f"{', dataset_number, ':02}"'])

    timestamp_path = PathJoinSubstitution(
        [
            dataset_path,
            "data_odometry_calib/dataset/sequences",
            dataset_number_padded,
            "times.txt",
        ]
    )
    poses_path = PathJoinSubstitution(
        [
            dataset_path,
            "data_odometry_poses/dataset/poses",
            PythonExpression(['f"{', dataset_number, ':02}.txt"']),
        ]
    )
    point_cloud_folder_path = PathJoinSubstitution(
        [
            dataset_path,
            "data_odometry_velodyne/dataset/sequences",
            dataset_number_padded,
            "velodyne",
        ]
    )
    urdf_filename = PythonExpression(['f"{', dataset_number, ':02}.urdf.xml"'])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="log_level",
                default_value="info",
                description="Log level for replayer",
            ),
            DeclareLaunchArgument(
                "dataset_number",
                default_value="00",
                description="Dataset number to load. Omit any leading zeros.",
            ),
            DeclareLaunchArgument(
                "dataset_path",
                default_value="/media/ltf/LTFUbuntuSSD/kitti_dataset",
                description="Path where all dataset folders are located",
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("ros2_kitti_description"),
                        "/load_urdf.launch.py",
                    ]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "urdf_filename": urdf_filename,
                    "launch_rviz": "False",
                }.items(),
            ),
            ComposableNodeContainer(
                name="replayer_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[
                    ComposableNode(
                        package="ros2_kitti_core",
                        plugin="r2k_core::KITTIReplayerNode",
                        name="kitti_replayer",
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "timestamp_path": timestamp_path,
                                "poses_path": poses_path,
                                "point_cloud_folder_path": point_cloud_folder_path,
                            }
                        ],
                    ),
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d"
                    + os.path.join(
                        get_package_share_directory("ros2_kitti_core"),
                        "rviz",
                        "visualize_ground_truth.rviz",
                    )
                ],
            ),
        ]
    )
