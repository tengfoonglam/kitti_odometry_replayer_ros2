import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_filename = LaunchConfiguration('urdf_filename', default='default.urdf.xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'urdf_filename',
            default_value='default.urdf.xml',
            description='Vehicle URDF to visualize'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[PathJoinSubstitution([
                    get_package_share_directory('ros2_kitti_description'),
                    urdf_filename])]),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d' + os.path.join(get_package_share_directory('ros2_kitti_description'),
                                    'visualize_vehicle.rviz')]
        )
    ])
