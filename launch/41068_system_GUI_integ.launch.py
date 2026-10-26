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

    ignition_share = get_package_share_directory(
        '41068_ignition_bringup'
    )

    vision_share = get_package_share_directory(
        'robot_vision'
    )

    # 1. Launch Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ignition_share,
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
            # 'parrot_camera_pitch': '1.5707963267948966' # 90 degrees in radians
            'parrot_camera_pitch': '1.2217304764',  # 70 degrees in radians
        }.items()
    )

    # 2. Launch both fire detectors
    fire_detection = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        vision_share,
                        'launch',
                        'fire_detection.launch.py'
                    )
                )
            )
        ]
    )

    # 3. Launch existing GUI
    gui_path = os.path.expanduser(
        '~/41068_ws/src/Robotics-Studio-1/'
        'GUI_Integration/GUI_Test2.py'
    )

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
        fire_detection,
        gui,
    ])