#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


REPO_ROOT = Path.home() / '41068_ws' / 'src' / 'Robotics-Studio-1'


def generate_launch_description():
    pkg = FindPackageShare('41068_ignition_bringup')

    patrol = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg, 'launch', '41068_coverage_patrol.launch.py'
            ])
        ),
        launch_arguments={
            'mode': 'attach',
            'altitude': LaunchConfiguration('altitude'),
            'repeat': LaunchConfiguration('repeat'),
        }.items(),
    )

    reconstruction = ExecuteProcess(
        cmd=[
            'python3',
            str(REPO_ROOT / 'scripts' / 'environment_reconstruction.py'),
            '--ros-args',
            '-r', '/tf:=/parrot1/tf',
            '-r', '/tf_static:=/parrot1/tf_static',
        ],
        output='screen',
        name='environment_reconstruction',
    )

    return LaunchDescription([
        DeclareLaunchArgument('altitude', default_value='10.0'),
        DeclareLaunchArgument('repeat', default_value='false'),
        lidar,
        reconstruction,
        patrol,
    ])
