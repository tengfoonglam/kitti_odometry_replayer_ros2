from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    dataset_number = LaunchConfiguration('dataset_number', default='00')
    dataset_path = LaunchConfiguration(
        'dataset_path', default=str(Path().home() / "kitti_dataset"))

    timestamp_path = PathJoinSubstitution(
        [dataset_path, "data_odometry_calib/dataset/sequences", dataset_number, "times.txt"])
    poses_path = PathJoinSubstitution(
        [dataset_path, "data_odometry_poses/dataset/poses",
         PythonExpression(['f"{', dataset_number, ':02}.txt"'])])
    point_cloud_folder_path = PathJoinSubstitution(
        [dataset_path, "data_odometry_velodyne/dataset/sequences", dataset_number, "velodyne"])
    urdf_filename = PythonExpression(['f"{', dataset_number, ':02}.urdf.xml"'])

    return LaunchDescription([
        DeclareLaunchArgument(name='log_level', default_value='info',
                              description='Log level for replayer'),
        DeclareLaunchArgument(
            'dataset_number',
            default_value='00',
            description='Dataset number to load. Must be a double digit number. e.g. 00, 10'),
        DeclareLaunchArgument(
            'dataset_path',
            default_value=str(Path().home() / "kitti_dataset"),
            description='Path where all dataset folders are located'),
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                get_package_share_directory("ros2_kitti_description"),
                '/visualize_urdf.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'urdf_filename': urdf_filename}.items()),
        Node(
            package='ros2_kitti_replay',
            executable='run_kitti_replayer',
            name='kitti_replayer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, "timestamp_path": timestamp_path,
                         "poses_path": poses_path,
                         "point_cloud_folder_path": point_cloud_folder_path}
                        ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        )
    ])