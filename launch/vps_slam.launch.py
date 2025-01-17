from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('vps_slam')

    # Path to parameters file
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Define the VPS SLAM node
    vps_slam_node = Node(
        package='vps_slam',
        executable='vps_slam_node',
        name='vps_slam',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('image', 'zedm/zed_node/left/image_rect_color'),
            ('depth', 'zedm/zed_node/depth/depth_registered'),
            ('camera_info', 'zedm/zed_node/left/camera_info'),
        ],
        arguments= ['--ros-args', '--log-level', 'debug']
    )

    return LaunchDescription([
        vps_slam_node
    ])