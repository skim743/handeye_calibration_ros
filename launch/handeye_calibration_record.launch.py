from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    piper_topic = LaunchConfiguration('piper_topic')

    aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('aruco_ros'), 'launch', 'single.launch.py'])),
        launch_arguments={
            'eye': LaunchConfiguration('eye'),
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size': LaunchConfiguration('marker_size'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('eye', default_value='left'),
        DeclareLaunchArgument('marker_id', default_value='100'),
        DeclareLaunchArgument('marker_size', default_value='0.1'),
        DeclareLaunchArgument('mode', default_value='eye_to_hand'),
        DeclareLaunchArgument('piper_topic', default_value='/end_pose'),
        aruco,
        Node(
            package='handeye_calibration_ros',
            executable='handeye_calibration_record',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mode': mode,
                'piper_topic': piper_topic,
            }],
        ),
    ])
