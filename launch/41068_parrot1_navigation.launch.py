"""Launch Parrot1 navigation bridge, optionally including the shared Gazebo simulation.

The bridge uses the existing /parrot1/navigate_to_pose Nav2 action server.
Do not start another Nav2 stack here when attaching to an existing simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _setup(context):
    package = FindPackageShare('41068_ignition_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    start_sim = LaunchConfiguration('start_sim').perform(context).lower() == 'true'
    actions = []

    if start_sim:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [package, 'launch', '41068_ignition.launch.py'])),
            launch_arguments={
                'husky': 'true',
                'parrot': 'true',
                'slam': 'true',
                'nav2': 'true',
                'rviz': 'false',
                'world': LaunchConfiguration('world'),
                'gz_gui': LaunchConfiguration('gz_gui'),
                'use_sim_time': str(use_sim_time).lower(),
            }.items(),
        ))

    actions.append(Node(
        package='41068_ignition_bringup',
        executable='parrot1_navigation.py',
        name='parrot_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'global_frame': LaunchConfiguration('global_frame'),
            'nav_action': '/parrot1/navigate_to_pose',
        }],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='false', choices=['true', 'false'],
                              description='true launches one shared Husky + Parrot Gazebo simulation.'),
        DeclareLaunchArgument('world', default_value='large_demo',
                              choices=['simple_trees', 'large_demo']),
        DeclareLaunchArgument('gz_gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('global_frame', default_value='map',
                              description='Must match Parrot1 Nav2 global frame.'),
        OpaqueFunction(function=_setup),
    ])
