from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
import os

def generate_launch_description():
    c920_config = os.path.join(
        get_package_share_directory('bluhmbot'),
        'config',
        'v4l2_camera.yaml')
    drive_config = os.path.join(
        get_package_share_directory('bluhmbot'),
        'config',
        'drivespecs.yaml')
    joystick_config = os.path.join(
        get_package_share_directory('bluhmbot'),
        'config',
        'joystick.yaml')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='zs_x11',
            executable='zs_x11',
            name='left_wheel',
            parameters=[drive_config]),
        launch_ros.actions.Node(
            package='zs_x11',
            executable='zs_x11',
            name='right_wheel',
            parameters=[drive_config]),
  ])

