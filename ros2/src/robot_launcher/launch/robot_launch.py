from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_launcher'),
        'config',
        'drivespecs.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='status_led',
            executable='status_led',
            name='status_led'),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node'),
        launch_ros.actions.Node(
            package='zs_x11',
            executable='zs_x11',
            name='zs_x11',
            parameters=[config]),
        launch_ros.actions.Node(
            package='l298n',
            executable='l298n',
            name='l298n'),
        launch_ros.actions.Node(
            package='motorcontroller',
            executable='motorcontroller',
            name='motorcontroller'),
        launch_ros.actions.Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            arguments=['-p', 'image_size:="[640,480]"',
                '-p', 'camera_frame_id:=camera_optical_link',
                '-p', 'brightness:=128'
                '-p', 'contrast:=128'
                '-p', 'saturation:=128'
                '-p', 'gain:=0'
                '-p', 'white_balance_temperature:=4000'
                '-p', 'sharpness:=128'
                '-p', 'backlight_compensation:=0'
                '-p', 'focus_absolute:=0'
                '-p', 'poom_absolute:=100']),
  ])

