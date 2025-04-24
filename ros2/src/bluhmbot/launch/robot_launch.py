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
            package='status_led',
            executable='status_led',
            name='status_led'),
        launch_ros.actions.Node(
            package='bluhmbot',
            executable='battery_management_node',
            name='battery_management_node'),
        launch_ros.actions.Node(
            package='lsm6ds3',
            executable='lsm6ds3_node',
            name='lsm6ds3_node'),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node'),
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joystick_config],
            ),
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
        launch_ros.actions.Node(
            package='l298n',
            executable='l298n',
            name='l298n'),
        launch_ros.actions.Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[c920_config],
            remappings=[
                ('/camera_info', '/camera/camera_info'),
                ('/image_raw', '/camera/image_raw'),
                ('/image_raw/compressed', '/camera/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/camera/image_raw/compressedDepth'),
                ('/image_raw/theora', '/camera/image_raw/theora'),
                ('/image_raw/zstd', '/camera/image_raw/zstd'),
            ]),
  ])

