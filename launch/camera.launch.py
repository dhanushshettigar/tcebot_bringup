from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from os.path import join

pkg_tcebot_bringup = get_package_share_directory('tcebot_bringup')

def generate_launch_description():
    intrinsic_params_file = DeclareLaunchArgument(
        'cam_params_file',
        default_value='file://' + join(pkg_tcebot_bringup, 'config', 'picam.yaml'),
        description='Path to camera intrinsics YAML file'
    )

    return LaunchDescription([
        intrinsic_params_file,
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [640, 480],
                'camera_frame_id': 'camera_link',
                'camera_info_url': LaunchConfiguration('cam_params_file'),
            }],
        )
    ])