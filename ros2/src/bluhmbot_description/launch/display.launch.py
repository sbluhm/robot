from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='bluhmbot_description').find('bluhmbot_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'bluhmbot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-entity', 'bluhmbot', '-topic', 'robot_description'],
      output='screen'
    )
    robot_localization_node = Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node',
      output='screen',
      parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz_node
    ])
