import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# Obtains share directory paths.
pkg_tcebot_bringup = get_package_share_directory('tcebot_bringup')
pkg_tcebot_control = get_package_share_directory('tcebot_control')
pkg_tcebot_description = get_package_share_directory('tcebot_description')

def generate_launch_description():
    # Declares launch arguments
    camera_arg = DeclareLaunchArgument(
            'include_camera',
            default_value='True',
            description='Indicates whether to include camera launch.')
    camera =  LaunchConfiguration('include_camera')
    rplidar_arg = DeclareLaunchArgument(
            'include_rplidar',
            default_value='True',
            description='Indicates whether to include rplidar launch.')
    rplidar =  LaunchConfiguration('include_rplidar')

    # Includes tcebot_description launch file
    include_tcebot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tcebot_description, 'launch', 'tcebot_description.launch.py'),
        ),
        launch_arguments={
            'rsp': 'True',
        }.items()
    )

    # Include tcebot_control launch file
    include_tcebot_control =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tcebot_control, 'launch', 'tcebot_control.launch.py'),
        ),
        launch_arguments={
        }.items()
    )

    # Include rplidar launch file
    include_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tcebot_bringup, 'launch', 'rplidar.launch.py'),
        ),
        launch_arguments={
            "serial_port": '/dev/ttyUSB0',
        }.items(),
                condition=IfCondition(rplidar)
    )
    # Include camera launch file
    include_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tcebot_bringup, 'launch', 'camera.launch.py'),
        ),
        launch_arguments={
        }.items(),
                condition=IfCondition(camera)
    )

    tcebot_control_timer = TimerAction(period=5.0, actions=[include_tcebot_control])
    # Defer sensors launch to avoid overhead while robot_state_publisher is setting up.
    rplidar_timer = TimerAction(period=3.0, actions=[include_rplidar])
    camera_timer = TimerAction(period=3.0, actions=[include_camera])

    return LaunchDescription([
        include_tcebot_description,
        tcebot_control_timer,
        camera_arg,
        camera_timer,
        rplidar_arg,
        rplidar_timer,
    ])