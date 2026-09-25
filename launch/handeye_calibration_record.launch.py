from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    piper_topic = LaunchConfiguration('piper_topic')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='eye_to_hand'),
        DeclareLaunchArgument('piper_topic', default_value='/end_pose'),
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
