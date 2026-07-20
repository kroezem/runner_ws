from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{'autorepeat_rate': 20.0, 'deadzone': 0.05}],
        ),
        Node(
            package='runner_teleop',
            executable='teleop_node',
            parameters=[{
                'axis_steer': 0,
                'axis_brake': 2,
                'axis_throttle': 5,
            }],
        ),
        Node(package='runner_motor', executable='motor_node'),
    ])
