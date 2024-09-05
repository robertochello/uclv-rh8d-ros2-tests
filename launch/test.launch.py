from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            output='screen',
            package='uclv_rh8d_ros2_tests',
            executable='test_moveFingerMotor',
            name='moveFingerMotor',
            parameters=[
            ]
        ),
        Node(
            output='screen',
            package='uclv_rh8d_ros2_tests',
            executable='sin_generator_node',
            name='sinGenerator',
            parameters=[
            ]
        ),
    ])
