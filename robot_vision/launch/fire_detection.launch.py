from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    for robot in ['husky1', 'parrot1']:
        nodes.append(
            Node(
                package='robot_vision',
                executable='fire_detector',
                name=f'{robot}_fire_detector',
                output='screen',
                parameters=[{
                    'image_topic': f'/{robot}/camera/image',
                    'fire_result_topic': f'/{robot}/vision/fire_detected',
                    'use_sim_time': True,
                }],
            )
        )

    return LaunchDescription(nodes)