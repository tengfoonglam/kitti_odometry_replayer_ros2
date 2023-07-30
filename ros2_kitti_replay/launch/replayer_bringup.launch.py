import os

# from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context: LaunchContext) -> LaunchDescription:
    # Replayer Launch Configurations
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    dataset_number = LaunchConfiguration("dataset_number", default="00")
    dataset_path = LaunchConfiguration(
        "dataset_path", default="/media/ltf/LTFUbuntuSSD/kitti_dataset"
    )
    ground_truth_data_frame_prefix = LaunchConfiguration(
        "ground_truth_data_frame_prefix", default="ground_truth"
    )

    # Odometry Node Launch Configurations
    odometry_data_frame_prefix = LaunchConfiguration(
        "odometry_data_frame_prefix", default="odometry"
    )
    odometry_package = LaunchConfiguration("odometry_package", default="")
    odometry_plugin = LaunchConfiguration("odometry_plugin", default="")
    odometry_config_path = LaunchConfiguration("odometry_config_path", default="")
    vehicle_sensor_link = LaunchConfiguration("vehicle_sensor_link", default="lidar")

    # Shared Launch Configurations
    odometry_data_frame_prefix = LaunchConfiguration(
        "odometry_data_frame_prefix", default="odometry"
    )

    # Check whether it is required to lauch the odometry node
    launch_odometry = PythonExpression(
        [
            'len("',
            odometry_package,
            '") > 0 and len("',
            odometry_plugin,
            '") > 0 and len("',
            vehicle_sensor_link,
            '") > 0',
        ]
    )

    # Check whether to override odometry data frame_id prefix

    # Set this False if you are running your own custom odometry node
    # (not launched with the replayer as a single component) and still want to
    # specify your own frame_id prefix
    OVERRIDE_ODOMETRY_DATA_PREFIX = True

    processed_odometry_data_frame_prefix = PythonExpression(
        [
            '"',
            ground_truth_data_frame_prefix,
            '" if ((not',
            launch_odometry,
            ") and ",
            OVERRIDE_ODOMETRY_DATA_PREFIX,
            ') else "',
            odometry_data_frame_prefix,
            '"',
        ]
    )

    # Paths of data files
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
    gray_image_folder_path = PathJoinSubstitution(
        [
            dataset_path,
            "data_odometry_gray/dataset/sequences",
            dataset_number_padded,
        ]
    )
    colour_image_folder_path = PathJoinSubstitution(
        [
            dataset_path,
            "data_odometry_color/dataset/sequences",
            dataset_number_padded,
        ]
    )
    urdf_filename = PythonExpression(['f"{', dataset_number, ':02}.urdf.xml"'])

    # Odometry node params
    VEHICLE_BASE_LINK = "p0"  # Fixed by URDF
    ODOMETRY_FRAME_ID = "odom"  # Fixed for convenience
    POINTCLOUD_TOPIC = "lidar_pc"  # Fixed in Replayer
    odometry_reference_frame_id = PathJoinSubstitution(
        [ground_truth_data_frame_prefix, VEHICLE_BASE_LINK]
    )
    odometry_base_link_frame_id = PathJoinSubstitution(
        [odometry_data_frame_prefix, VEHICLE_BASE_LINK]
    )
    odometry_sensor_link_frame_id = PathJoinSubstitution(
        [odometry_data_frame_prefix, vehicle_sensor_link]
    )

    # Set which RVIZ file to launch
    visualize_ground_truth_rviz_path = os.path.join(
        get_package_share_directory("ros2_kitti_replay"),
        "rviz",
        "visualize_ground_truth.rviz",
    )

    visualize_odometry_run_rviz_path = os.path.join(
        get_package_share_directory("ros2_kitti_replay"),
        "rviz",
        "visualize_run.rviz",
    )

    rviz_path_arg = PythonExpression(
        [
            '"-d"+(',
            '"',
            visualize_odometry_run_rviz_path,
            '" if ',
            launch_odometry,
            ' else "',
            visualize_ground_truth_rviz_path,
            '")',
        ]
    )

    # Create Compositions with and without odometry node
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
                "gray_image_folder_path": gray_image_folder_path,
                "colour_image_folder_path": colour_image_folder_path,
                "ground_truth_data_frame_prefix": processed_odometry_data_frame_prefix,
                "odometry_data_frame_prefix": odometry_data_frame_prefix,
                "odometry_reference_frame_id": odometry_reference_frame_id,
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
                "odometry_frame_id": ODOMETRY_FRAME_ID,
                "pointcloud_topic": POINTCLOUD_TOPIC,
                "base_link_frame_id": odometry_base_link_frame_id,
                "sensor_frame_id": odometry_sensor_link_frame_id,
                "config_path": odometry_config_path,
            }
        ],
    )

    return LaunchDescription(
        [
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
                    "frame_prefix": ground_truth_data_frame_prefix,
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
                    "frame_prefix": odometry_data_frame_prefix,
                    "static_frame_base_frame_id": "odom",
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
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="rviz2",
                        namespace="",
                        executable="rviz2",
                        name="rviz2",
                        parameters=[{"use_sim_time": use_sim_time}],
                        arguments=[rviz_path_arg],
                    ),
                ],
            ),
        ]
    )


def generate_launch_description() -> LaunchDescription:
    return [
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
            "ground_truth_data_frame_prefix",
            default_value="ground_truth",
            description="Prefix prepended to the frame_id of the ground truth pose message",
        ),
        DeclareLaunchArgument(
            "odometry_data_frame_prefix",
            default_value="odometry",
            description="Prefix prepended to the frame_id of the published data topics such as"
            "point clouds, images, etc. "
            "Note: Only used then a odometry node is specified to be launched. Otherwise, "
            "the odometry data topics will be prepended with the ground_truth_data_frame_prefix "
            "instead. This is so that the data topics are visualized correctly "
            "on the ground truth vehicle when an odometry node is not running. "
            "If you want to disable this automatic overriding of the frame prefix, set the "
            "variable OVERRIDE_ODOMETRY_DATA_PREFIX in the launch file to False",
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
            "vehicle_sensor_link",
            default_value="lidar",
            description="Name of the link which the odometry sensor is "
            "located (e.g. lidar, p0)"
            "Leave empty if launching odometry component is not required",
        ),
        DeclareLaunchArgument(
            "odometry_config_path",
            default_value="",
            description="Path config file used for the odometry component. "
            "Leave empty if launching odometry component is not required",
        ),
        OpaqueFunction(function=launch_setup),
    ]
