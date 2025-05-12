import launch
import launch_ros.actions
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('bluhmbot')
    pkg_share = get_package_share_directory('bluhmbot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_model_path = os.path.join(pkg_share, 'src', 'description', 'bluhmbot_description.sdf')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    c920_config = os.path.join(
        bringup_dir, 'config', 'v4l2_camera.yaml')
    drive_config = os.path.join(
        bringup_dir, 'config', 'drivespecs.yaml')
    joystick_config = os.path.join(
        bringup_dir, 'config', 'joystick.yaml')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'tb3_sandbox.yaml'),
        description='Full path to map yaml file to load'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
    )

    return launch.LaunchDescription([
        declare_namespace_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_log_level_cmd,
        joint_state_publisher_node,
        robot_state_publisher_node,
        Node(
            package='status_led',
            executable='status_led',
            name='status_led'),
        Node(
            package='bluhmbot',
            executable='battery_management_node',
            name='battery_management_node'),
        Node(
            package='lsm6ds3',
            executable='lsm6ds3_node',
            name='lsm6ds3_node'),
        Node(
            package='bluhmbot',
            executable='twist_to_motors_node',
            name='twist_to_motors_node',
            parameters=[drive_config]),
        Node(
            package='bluhmbot',
            executable='diff_tf_node',
            name='diff_tf_node'),
        Node(
            package='zs_x11',
            executable='zs_x11',
            name='left_wheel',
            parameters=[drive_config]),
        Node(
            package='zs_x11',
            executable='zs_x11',
            name='right_wheel',
            parameters=[drive_config]),
        Node(
            package='l298n',
            executable='l298n',
            name='l298n'),
        Node(
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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0.15', '0', '0', '0', 'odom', 'map'],
            ),
  ])

