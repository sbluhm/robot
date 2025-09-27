from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('bluhmbot')
    urdf_path = os.path.join(pkg, 'urdf', 'test_zk_bm1.urdf')
    controllers_yaml = os.path.join(pkg, 'config', 'diffbot_controllers.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()


    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        # optional: name='robot_state_publisher'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        arguments=['--ros-args',
               '--log-level', 'controller_manager:=debug',
               '--log-level', 'pluginlib.ClassLoader:=debug',
               '--log-level', 'class_loader.ClassLoader:=debug'],

        parameters=[controllers_yaml]
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    brush_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['brush_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    cutter_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['cutter_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    return LaunchDescription([rsp, control_node, jsb_spawner, brush_spawner, cutter_spawner])
