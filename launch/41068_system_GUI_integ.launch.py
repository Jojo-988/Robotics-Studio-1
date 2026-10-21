#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory(
        '41068_ignition_bringup'
    )

    # Launch existing Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share,
                'launch',
                '41068_ignition.launch.py'
            )
        ),
        launch_arguments={
            'husky': 'true',
            'parrot': 'true',
            'slam': 'false',
            'nav2': 'false',
            'rviz': 'false',
            'world': 'large_demo',
        }.items()
    )

    # Existing GUI Python script
    gui_path = os.path.expanduser(
        '~/41068_ws/src/Robotics-Studio-1/'
        'GUI_Integration/GUI_Test2.py'
    )

    # Start GUI after Gazebo has had time to initialise
    gui = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', gui_path],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        gui,
    ])
