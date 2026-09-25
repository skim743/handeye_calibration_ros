from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    samples_file = LaunchConfiguration('samples_file')
    mode = LaunchConfiguration('mode')
    piper_topic = LaunchConfiguration('piper_topic')
    settle_time = LaunchConfiguration('settle_time')
    approach_offset = LaunchConfiguration('approach_offset')

    return LaunchDescription([
        DeclareLaunchArgument(
            'samples_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('handeye_calibration_ros'), 'calibration_files',
                '2026-09-23_19-04-23_samples.json'])),
        DeclareLaunchArgument('mode', default_value='eye_to_hand'),
        DeclareLaunchArgument('piper_topic', default_value='/end_pose'),
        DeclareLaunchArgument('settle_time', default_value='5.0'),
        DeclareLaunchArgument('approach_offset', default_value='0.0'),
        Node(
            package='handeye_calibration_ros',
            executable='handeye_calibration_auto',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'samples_file': samples_file,
                'mode': mode,
                'piper_topic': piper_topic,
                'settle_time': ParameterValue(settle_time, value_type=float),
                'approach_offset': ParameterValue(approach_offset, value_type=float),
            }],
        ),
    ])
