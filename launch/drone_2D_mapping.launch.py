"""Launch the bird's-eye mapper against an already running drone."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('41068_ignition_bringup')
    )
    default_config = (
        package_share / 'config' / 'drone_2d_mapping.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=str(default_config),
        ),
        Node(
            package='41068_ignition_bringup',
            executable='drone_cloud_diagnostic.py',
            name='drone_cloud_diagnostic',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('/tf', '/parrot1/tf'),
                ('/tf_static', '/parrot1/tf_static'),
            ],
        ),
    ])