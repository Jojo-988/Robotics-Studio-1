
#!/usr/bin/env python3
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ignition_share = get_package_share_directory(
        '41068_ignition_bringup'
    )

    vision_share = get_package_share_directory(
        'robot_vision'
    )

    repo_root = Path(__file__).resolve().parent.parent

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
            'husky': 'false',
            'parrot': 'true',
            'slam': 'false',
            'nav2': 'false',
            'rviz': 'false',
            'world': 'large_demo',
            'parrot_camera_pitch': '1.2217304764',
        }.items()
    )

    # 2. Launch LiDAR handler
    lidar = ExecuteProcess(
        cmd=[
            'python3',
            str(repo_root / 'scripts' / 'lidar_handler.py')
        ],
        output='screen',
        name='lidar_handler'
    )

    # 3. Launch fire detectors
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

    # 4. Launch vegetation detector
    vegetation_detector = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='robot_vision',
                executable='vegetation_detector',
                name='parrot_vegetation_detector',
                output='screen',
                parameters=[{
                    'image_topic': '/parrot1/camera/image',
                    'depth_topic': '/parrot1/camera/depth/image',
                    'odom_topic': '/parrot1/odometry',
                    'use_sim_time': True,
                }]
            )
        ]
    )

    # 5. Launch GUI
    gui = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    str(
                        repo_root
                        / 'GUI_Integration'
                        / 'GUI_Test2.py'
                    )
                ],
                output='screen',
                name='forest_monitoring_gui'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        lidar,
        fire_detection,
        vegetation_detector,
        gui
    ])
