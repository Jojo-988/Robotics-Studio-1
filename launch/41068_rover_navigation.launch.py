"""Husky goal navigation with optional reuse of an existing Nav2 stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _setup(context):
    package = FindPackageShare('41068_ignition_bringup')

    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    )

    start_sim = (
        LaunchConfiguration('start_sim').perform(context).lower() == 'true'
    )

    reuse_nav2 = (
        LaunchConfiguration('reuse_nav2').perform(context).lower() == 'true'
    )

    external = (
        LaunchConfiguration('controller').perform(context) == 'external'
    )

    params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key='husky1',
            param_rewrites={
                'use_sim_time': str(use_sim_time).lower()
            },
            convert_types=True,
        )
    )

    remaps = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/map', 'map'),
    ]

    actions = []

    # ==========================================================
    # OPTIONAL GAZEBO SIMULATION
    # ==========================================================

    if start_sim:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        package,
                        'launch',
                        '41068_ignition.launch.py',
                    ])
                ),
                launch_arguments={
                    'husky': 'true',
                    'parrot': 'false',
                    'slam': 'true',
                    'nav2': 'false',
                    'rviz': 'false',
                    'world': LaunchConfiguration('world'),
                    'gz_gui': LaunchConfiguration('gz_gui'),
                    'use_sim_time': str(use_sim_time).lower(),
                }.items(),
            )
        )

    # ==========================================================
    # NAV2 SERVERS
    #
    # reuse_nav2 = false:
    #   Start dedicated Husky planner/controller/lifecycle manager.
    #
    # reuse_nav2 = true:
    #   Reuse Nav2 servers already started by the combined launch.
    # ==========================================================

    if not reuse_nav2:

        managed = ['planner_server']

        actions.append(
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace='husky1',
                output='screen',
                parameters=[params],
                remappings=remaps,
            )
        )

        if not external:
            managed.append('controller_server')

            actions.append(
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    namespace='husky1',
                    output='screen',
                    parameters=[params],
                    remappings=remaps + [
                        ('cmd_vel', 'rover/nav2_cmd_vel')
                    ],
                )
            )

        actions.append(
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_rover',
                namespace='husky1',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': managed,
                }],
            )
        )

    # ==========================================================
    # CUSTOM ROVER NAVIGATION
    #
    # Always launch this node, including when reusing Nav2.
    # ==========================================================

    actions.append(
        Node(
            package='41068_ignition_bringup',
            executable='rover_goal_navigation.py',
            name='rover_navigation',
            namespace='husky1',
            output='screen',
            parameters=[
                params,
                {
                    'use_sim_time': use_sim_time,
                    'controller_action': (
                        'avoidance/follow_path'
                        if external else 'follow_path'
                    ),
                    'controller_cmd_topic': (
                        'avoidance/cmd_vel'
                        if external else (
                            'cmd_vel_nav' if reuse_nav2
                            else 'rover/nav2_cmd_vel'
                        )
                    ),
                    'require_avoidance_heartbeat': external,
                },
            ],
            remappings=remaps,
        )
    )

    return actions


def generate_launch_description():

    package = FindPackageShare('41068_ignition_bringup')

    return LaunchDescription([

        DeclareLaunchArgument(
            'start_sim',
            default_value='true',
            choices=['true', 'false'],
            description='Start Gazebo or attach to an existing simulation.',
        ),

        DeclareLaunchArgument(
            'reuse_nav2',
            default_value='false',
            choices=['true', 'false'],
            description='Reuse existing Husky Nav2 servers.',
        ),

        DeclareLaunchArgument(
            'controller',
            default_value='nav2',
            choices=['nav2', 'external'],
            description='Use Nav2 or external FollowPath controller.',
        ),

        DeclareLaunchArgument(
            'world',
            default_value='large_demo',
            choices=['simple_trees', 'large_demo'],
        ),

        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            choices=['true', 'false'],
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                package,
                'config',
                'rover_navigation.yaml',
            ]),
        ),

        OpaqueFunction(function=_setup),
    ])