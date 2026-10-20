#!/usr/bin/env python3
import math
import uuid

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from patrol.preflight import check_environment


def _launch(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value('mode')
    legacy = value('start_sim').lower()
    if legacy != 'auto':
        if legacy not in ('true', 'false'):
            raise ValueError('start_sim must be auto, true or false')
        mode = 'standalone' if legacy == 'true' else 'attach'
    if value('use_sim_time').lower() != 'true':
        raise ValueError('Coverage simulation requires use_sim_time:=true so sensor timestamps share /clock.')
    errors, notes = check_environment(mode)
    if errors:
        raise RuntimeError('Preflight failed: ' + '; '.join(errors)
                           + '. Run rosdep install --from-paths src --ignore-src -r -y, then rebuild.')
    actions = [LogInfo(msg='[preflight] ' + note) for note in notes]
    package = FindPackageShare('41068_ignition_bringup')
    if mode == 'standalone':
        for name in ('spawn_x', 'spawn_y', 'spawn_z'):
            if not math.isfinite(float(value(name))):
                raise ValueError(name + ' must be finite')
        partition = value('partition')
        if partition == 'auto':
            partition = 'patrol_' + uuid.uuid4().hex[:12]
        if not partition.strip():
            raise ValueError('partition must not be empty')
        actions += [SetEnvironmentVariable('IGN_PARTITION', partition),
                    SetEnvironmentVariable('GZ_PARTITION', partition),
                    LogInfo(msg='[preflight] Gazebo partition: ' + partition)]
        rendering = value('rendering')
        if rendering != 'auto':
            actions.append(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE',
                                                  '1' if rendering == 'software' else '0'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([package, 'launch', '41068_ignition.launch.py'])),
            launch_arguments={
                'husky': 'false', 'parrot': 'true', 'slam': 'false', 'nav2': 'false', 'rviz': 'false',
                'world': value('world'), 'use_sim_time': 'true', 'gz_gui': value('gz_gui'),
                'parrot_camera_pitch': '1.5707963267948966',
                'parrot_x': value('spawn_x'), 'parrot_y': value('spawn_y'), 'parrot_z': value('spawn_z'),
            }.items()))
    else:
        actions.append(LogInfo(msg='[preflight] Attach mode: using existing ROS domain, robot and bridges. '
                               'Disable other Parrot cmd_vel controllers before attaching.'))
    patrol_node = Node(
        package='41068_ignition_bringup', executable='coverage_patrol.py',
        namespace='parrot1', name='coverage_patrol', output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': True,
            'repeat': ParameterValue(LaunchConfiguration('repeat'), value_type=bool),
            'altitude': ParameterValue(LaunchConfiguration('altitude'), value_type=float),
        }])
    actions += [RegisterEventHandler(OnProcessExit(
        target_action=patrol_node,
        on_exit=[EmitEvent(event=Shutdown(reason='Patrol controller exited'))])), patrol_node]
    return actions


def generate_launch_description():
    ld = LaunchDescription()
    for name, default, choices, description in (
        ('mode', 'standalone', ['standalone', 'attach'], 'Create a simulation or attach to an existing robot.'),
        ('start_sim', 'auto', ['auto', 'true', 'false'], 'Legacy override: false selects attach; prefer mode.'),
        ('world', 'large_demo', ['large_demo', 'simple_trees'], 'Gazebo world name.'),
        ('use_sim_time', 'true', None, 'Simulation clock; must be true for this launch.'),
        ('gz_gui', 'true', None, 'Show Gazebo window; false starts only the server.'),
        ('repeat', 'false', None, 'Repeat passes instead of returning home.'),
        ('altitude', '15.0', None, 'Survey altitude above world z=0, metres.'),
        ('spawn_x', '0.0', None, 'Standalone takeoff column, world X.'),
        ('spawn_y', '-8.0', None, 'Standalone takeoff column, world Y; clear of the original near-clip obstruction.'),
        ('spawn_z', '0.8', None, 'Standalone initial height, world Z.'),
        ('partition', 'auto', None, 'Standalone Gazebo partition; auto creates a fresh session.'),
        ('rendering', 'auto', ['auto', 'software', 'hardware'], 'auto preserves the graphics environment.'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, choices=choices, description=description))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=PathJoinSubstitution([
            FindPackageShare('41068_ignition_bringup'), 'config', 'coverage_patrol.yaml']),
        description='Coverage bounds, readiness, sensor validation and controller settings.'))
    ld.add_action(OpaqueFunction(function=_launch))

    return ld
