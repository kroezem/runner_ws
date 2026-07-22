# Single top-level launch for production teleoperation plus mapping.
# Do not run it concurrently with drive.launch.py, map.launch.py,
# sensors.launch.py, or any standalone localization launch: doing so duplicates
# hardware drivers or localization publishers. The RF2O A/B diagnostic launch
# is separate and must not run concurrently with this production stack.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('runner_bringup'),
        'launch',
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'sensors.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'teleop.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization.launch.py'))),
    ])
