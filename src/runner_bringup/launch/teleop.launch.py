from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node'),
        Node(package='runner_teleop', executable='teleop_node'),
        Node(package='runner_motor', executable='motor_node'),
    ])
