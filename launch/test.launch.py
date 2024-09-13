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
                {"motor_ids": [34]}
                # {"motor_ids": [34,35]}
                # {"motor_ids": [34,35,36]}
                # {"motor_ids": [34,35,36,37]}
                # {"motor_ids": [34,35,36,37,38]}
            ]
        ),
        Node(
            output='screen',
            package='uclv_rh8d_ros2_tests',
            executable='sin_generator_node',
            name='sinGenerator',
            parameters=[
                {"amplitude": 1000.0},
                {"frequency": 2.0},
                {"offset": 1000.0}
            ]
        ),
    ])
