import os

from pathlib import Path

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
from typing import List

POINT_CLOUD_FOLDER_AVAILABLE = True
GRAY_IMAGES_FOLDER_AVAILABLE = True
COLOUR_IMAGES_FOLDER_AVAILABLE = True
DATASET_PATH = str(Path().home() / "kitti_dataset")


def launch_setup(context: LaunchContext) -> List[LaunchDescription]:
    # Replayer Launch Configurations
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    start_time = LaunchConfiguration("start_time", default="0.0")
    end_time = LaunchConfiguration("end_time", default="0.0")

    dataset_number = LaunchConfiguration("dataset_number", default="00")
    dataset_path = LaunchConfiguration("dataset_path", default=DATASET_PATH)
    ground_truth_data_frame_prefix = LaunchConfiguration(
        "ground_truth_data_frame_prefix", default="ground_truth"
    )
    enable_point_cloud = LaunchConfiguration(
        "enable_point_cloud", default=str(POINT_CLOUD_FOLDER_AVAILABLE)
    )
    enable_gray_images = LaunchConfiguration(
        "enable_gray_images", default=str(GRAY_IMAGES_FOLDER_AVAILABLE)
    )
    enable_colour_images = LaunchConfiguration(
        "enable_colour_images", default=str(COLOUR_IMAGES_FOLDER_AVAILABLE)
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

    if OVERRIDE_ODOMETRY_DATA_PREFIX:
        processed_odometry_data_frame_prefix = PythonExpression(
            [
                '"',
                odometry_data_frame_prefix,
                '" if ',
                launch_odometry,
                ' else "',
                ground_truth_data_frame_prefix,
                '"',
            ]
        )
    else:
        processed_odometry_data_frame_prefix = odometry_data_frame_prefix

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
    P0_IMAGE_TOPIC = "p0_img"  # Fixed in Replayer
    P1_IMAGE_TOPIC = "p1_img"  # Fixed in Replayer
    P2_IMAGE_TOPIC = "p2_img"  # Fixed in Replayer
    P3_IMAGE_TOPIC = "p3_img"  # Fixed in Replayer

    odometry_base_link_frame_id = PathJoinSubstitution(
        [odometry_data_frame_prefix, VEHICLE_BASE_LINK]
    )
    odometry_sensor_link_frame_id = PathJoinSubstitution(
        [odometry_data_frame_prefix, vehicle_sensor_link]
    )

    pointcloud_topic = PythonExpression(
        ['"', POINTCLOUD_TOPIC, '" if "', enable_point_cloud, '" == "true" else ""']
    )
    p0_image_topic = PythonExpression(
        ['"', P0_IMAGE_TOPIC, '" if "', enable_gray_images, '" == "true" else ""']
    )
    p1_image_topic = PythonExpression(
        ['"', P1_IMAGE_TOPIC, '" if "', enable_gray_images, '" == "true" else ""']
    )
    p2_image_topic = PythonExpression(
        ['"', P2_IMAGE_TOPIC, '" if "', enable_colour_images, '" == "true" else ""']
    )
    p3_image_topic = PythonExpression(
        ['"', P3_IMAGE_TOPIC, '" if "', enable_colour_images, '" == "true" else ""']
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
                "ground_truth_data_frame_prefix": ground_truth_data_frame_prefix,
                "odometry_data_frame_prefix": processed_odometry_data_frame_prefix,
                "publish_point_cloud": enable_point_cloud,
                "publish_gray_images": enable_gray_images,
                "publish_colour_images": enable_colour_images,
                "start_time": start_time,
                "end_time": end_time,
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
                "pointcloud_topic": pointcloud_topic,
                "p0_img_topic": p0_image_topic,
                "p1_img_topic": p1_image_topic,
                "p2_img_topic": p2_image_topic,
                "p3_img_topic": p3_image_topic,
                "base_link_frame_id": odometry_base_link_frame_id,
                "sensor_frame_id": odometry_sensor_link_frame_id,
                "config_path": odometry_config_path,
            }
        ],
    )

    return [
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


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_number",
                default_value="00",
                description="Dataset number to load. Omit any leading zeros.",
            ),
            DeclareLaunchArgument(
                "dataset_path",
                default_value=DATASET_PATH,
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
                "Note: Only used then a odometry node is specified to be launched. Otherwise, the "
                "odometry data topics will be prepended with the ground_truth_data_frame_prefix "
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
            DeclareLaunchArgument(
                "enable_point_cloud",
                default_value=str(POINT_CLOUD_FOLDER_AVAILABLE),
                description="Replayer publishes and odometry node "
                "(if configured to load) subscribes to the point cloud",
            ),
            DeclareLaunchArgument(
                "enable_gray_images",
                default_value=str(GRAY_IMAGES_FOLDER_AVAILABLE),
                description="Replayer publishes and odometry node "
                "(if configured to load) subscribes to the gray images (p0 and p1)",
            ),
            DeclareLaunchArgument(
                "enable_colour_images",
                default_value=str(COLOUR_IMAGES_FOLDER_AVAILABLE),
                description="Replayer publishes and odometry node "
                "(if configured to load) subscribes to the colour images (p2 and p3)",
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
            OpaqueFunction(function=launch_setup),
        ]
    )
