from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    piper_topic = LaunchConfiguration('piper_topic')

    # Camera USB port depends on the setup: fixed camera (eye_to_hand) vs wrist camera (eye_in_hand)
    usb_port_id = PythonExpression([
        "'2-9' if '", mode, "' == 'eye_to_hand' else '2-10'"])
    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        parameters=[{
            'rgb_camera.color_profile': '1280x720x30',
            '_usb_port_id': ParameterValue(usb_port_id, value_type=str),
            'pointcloud.enable': True,
            'pointcloud.stream_filter': 2,
            'spatial_filter.enable': True,
            'temporal_filter.enable': True,
        }],
        # Feed the color stream to aruco_ros single.launch.py (eye:=left)
        remappings=[
            ('/camera/camera/color/image_raw', '/stereo/left/image_rect_color'),
            ('/camera/camera/color/camera_info', '/stereo/left/camera_info'),
        ],
    )

    piper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('piper'), 'launch', 'start_single_piper.launch.py'])),
    )

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
        # ros2 launch does not give nodes keyboard input, so the interactive node gets its own window.
        # --wait ties the window to the launch process; the trailing read keeps the window open after
        # exit so the calibration result stays readable. The node command is appended as "$@".
        DeclareLaunchArgument('terminal_prefix',
                              default_value='gnome-terminal --wait --title=handeye_calibration -- '
                                            'bash -c \'"$@"; read -p "Press Enter to close"\' _',
                              description='terminal the calibration node runs in'),
        DeclareLaunchArgument('start_delay', default_value='5.0',
                              description='s to wait for camera/arm/aruco before starting calibration'),
        DeclareLaunchArgument('eye', default_value='left'),
        DeclareLaunchArgument('marker_id', default_value='100'),
        DeclareLaunchArgument('marker_size', default_value='0.1'),
        DeclareLaunchArgument('mode', default_value='eye_to_hand'),
        DeclareLaunchArgument('piper_topic', default_value='/end_pose'),
        camera,
        piper,
        aruco,
        # Start last so camera, arm and aruco topics are up before the first sample
        TimerAction(
            period=LaunchConfiguration('start_delay'),
            actions=[
                Node(
                    package='handeye_calibration_ros',
                    executable='handeye_calibration_record',
                    prefix=LaunchConfiguration('terminal_prefix'),
                    output='screen',
                    parameters=[{
                        'mode': mode,
                        'piper_topic': piper_topic,
                    }],
                ),
            ],
        ),
    ])
