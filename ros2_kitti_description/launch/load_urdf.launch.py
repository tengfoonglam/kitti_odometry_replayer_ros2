import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    urdf_filename = LaunchConfiguration("urdf_filename", default="default.urdf.xml")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")
    frame_prefix = LaunchConfiguration("frame_prefix", default="")
    static_frame_base_frame_id = LaunchConfiguration(
        "static_frame_base_frame_id", default="map"
    )
    frame_prefix_with_slash = PythonExpression(
        ['"', frame_prefix, '/" if len("', frame_prefix, '")>0 else ""']
    )
    p0_frame_id = PythonExpression(['"', frame_prefix_with_slash, 'p0"'])
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "urdf_filename",
                default_value="default.urdf.xml",
                description="Vehicle URDF to visualize",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RVIZ Visualization. Note: Only for basic visualization when "
                "frame_prefix is empty",
            ),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value="",
                description="Prefix to the tf frame names of the launched URDF",
            ),
            DeclareLaunchArgument(
                "static_frame_base_frame_id",
                default_value="map",
                description="Base frame id for the urdf",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--qx",
                    "0.0",
                    "--qy",
                    "0.0",
                    "--qz",
                    "0.0",
                    "--qw",
                    "1.0",
                    "--frame-id",
                    static_frame_base_frame_id,
                    "--child-frame-id",
                    p0_frame_id,
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=frame_prefix,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "frame_prefix": frame_prefix_with_slash,
                    }
                ],
                arguments=[
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("ros2_kitti_description"),
                            urdf_filename,
                        ]
                    )
                ],
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(launch_rviz),
                arguments=[
                    "-d"
                    + os.path.join(
                        get_package_share_directory("ros2_kitti_description"),
                        "visualize_vehicle.rviz",
                    )
                ],
            ),
        ]
    )
