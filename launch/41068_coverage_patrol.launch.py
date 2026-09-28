
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('41068_ignition_bringup')
    ld = LaunchDescription()
    for name, default, description in (
        ('start_sim', 'true', 'Start Gazebo and Parrot. false attaches to an existing simulation.'),
        ('world', 'large_demo', 'Gazebo world name.'),
        ('use_sim_time', 'true', 'Use Gazebo clock.'),
        ('gz_gui', 'true', 'Show Gazebo window; false starts only the server.'),
        ('repeat', 'false', 'Repeat coverage passes instead of returning home.'),
        ('altitude', '15.0', 'Survey altitude above world z=0, metres.'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=PathJoinSubstitution([package, 'config', 'coverage_patrol.yaml']),
        description='Coverage bounds, footprint and controller settings.'))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([package, 'launch', '41068_ignition.launch.py'])),
        condition=IfCondition(LaunchConfiguration('start_sim')),
        launch_arguments={
            'husky': 'false', 'parrot': 'true', 'slam': 'false', 'nav2': 'false', 'rviz': 'false',
            'world': LaunchConfiguration('world'), 'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gz_gui': LaunchConfiguration('gz_gui'),
            'parrot_camera_pitch': '1.5707963267948966',
        }.items(),
    ))
    ld.add_action(Node(
        package='41068_ignition_bringup', executable='coverage_patrol.py',
        namespace='parrot1', name='coverage_patrol', output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            'repeat': ParameterValue(LaunchConfiguration('repeat'), value_type=bool),
            'altitude': ParameterValue(LaunchConfiguration('altitude'), value_type=float),
        }],
    ))
    return ld
