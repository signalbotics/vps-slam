from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('vps_slam')

    # Path to parameters file
    params_file = os.path.join(pkg_share, 'params', 'params.yaml')
    return LaunchDescription([

        DeclareLaunchArgument(
                "Parameter_launch_argument", default_value=TextSubstitution(text=str("Parameter_launch")),
                description="Parameter launch default value"
            ),

        DeclareLaunchArgument(
            "log_level",
            default_value = TextSubstitution(text=str("DEBUG")),
            description="Logging level"
        ),


        # Define the VPS SLAM node
        Node(
            package='vps_slam',
            executable='vps_slam_node',
            name='vps_slam_node',
            output='screen',
            remappings=[
                ('image', '/zedm/zed_node/left/image_rect_color'),
                ('depth', '/zedm/zed_node/depth/depth_registered'),
                ('camera_info', '/zedm/zed_node/left/camera_info'),
                ('gps', 'gps/fix'),
            ],
            arguments= ['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[params_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'config.rviz')],
        ),

    ])