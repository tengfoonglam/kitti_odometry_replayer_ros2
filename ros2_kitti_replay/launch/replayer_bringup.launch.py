import os

# from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    ground_truth_namespace = LaunchConfiguration(
        "ground_truth_namespace", default="ground_truth"
    )
    odometry_namespace = LaunchConfiguration("odometry_namespace", default="odometry")
    odometry_package = LaunchConfiguration("odometry_package", default="")
    odometry_plugin = LaunchConfiguration("odometry_plugin", default="")
    odometry_vehicle_frame = LaunchConfiguration("odometry_vehicle_frame", default="")
    odometry_frame_id = PythonExpression(
        ['f"{', odometry_namespace, "}/{", odometry_vehicle_frame, '}"']
    )
    global_frame_id = LaunchConfiguration("global_frame_id", default="map")

    replayer_component = ComposableNode(
        package="ros2_kitti_replay",
        plugin="r2k_replay::KITTIReplayerNode",
        name="kitti_replayer",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "timestamp_path": timestamp_path,
                "poses_path": poses_path,
                "point_cloud_folder_path": point_cloud_folder_path,
                "ground_truth_namespace": ground_truth_namespace,
                "odometry_namespace": odometry_namespace,
                "odometry_frame_id": odometry_frame_id,
            }
        ],
    )

    odometry_component = ComposableNode(
        package=odometry_package,
        plugin=odometry_plugin,
        name="odometry",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "global_frame_id": global_frame_id,
                "pointcloud_topic": "lidar_pc",
            }
        ],
    )

    launch_odometry = PythonExpression(
        [
            'len("',
            odometry_package,
            '") > 0 and len("',
            odometry_plugin,
            '") > 0 and len("',
            odometry_vehicle_frame,
            '") > 0',
        ]
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                "ground_truth_namespace",
                default_value="ground_truth",
                description="Namespace used for the ground truth pose topic",
            ),
            DeclareLaunchArgument(
                "odometry_namespace",
                default_value="odometry",
                description="Namespace used for the data topics (point clouds, images, etc)",
            ),
            DeclareLaunchArgument(
                "odometry_package",
                default_value="",
                description="Package in which odometry component is located. "
                "Leave empty if launching odometry component is not required",
            ),
            DeclareLaunchArgument(
                "odometry_plugin",
                default_value="",
                description="Name of odometry component to launch. "
                "Leave empty if launching odometry component is not required",
            ),
            DeclareLaunchArgument(
                "odometry_vehicle_frame",
                default_value="",
                description="Frame on the vehicle that is the odometry frame. "
                "Leave empty if launching odometry component is not required",
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
                    "frame_prefix": ground_truth_namespace,
                }.items(),
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
                    "frame_prefix": odometry_namespace,
                }.items(),
                condition=IfCondition(launch_odometry),
            ),
            ComposableNodeContainer(
                name="replayer_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[replayer_component],
                output="screen",
                condition=IfCondition(PythonExpression(["not ", launch_odometry])),
            ),
            ComposableNodeContainer(
                name="replayer_with_odometry_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[replayer_component, odometry_component],
                output="screen",
                condition=IfCondition(launch_odometry),
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d"
                    + os.path.join(
                        get_package_share_directory("ros2_kitti_replay"),
                        "rviz",
                        "visualize_run.rviz",
                    )
                ],
            ),
        ]
    )
