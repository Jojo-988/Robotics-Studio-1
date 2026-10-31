"""Camera mapping only: reuse the running simulation; never command motion."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('41068_ignition_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share/'config/camera_2d_mapping.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_camera_info_bridge', default_value='true',
                              description='Set false if another bridge publishes CameraInfo'),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='camera_mapping_info_bridge', output='screen',
            condition=IfCondition(LaunchConfiguration('start_camera_info_bridge')),
            arguments=['/model/parrot1/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
            remappings=[('/model/parrot1/camera/camera_info', '/parrot1/camera/camera_info')],
        ),
        Node(
            package='41068_ignition_bringup', executable='camera_bird_mapper.py',
            name='camera_bird_mapper', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            remappings=[('/tf', '/parrot1/tf'), ('/tf_static', '/parrot1/tf_static')],
        ),
    ])
