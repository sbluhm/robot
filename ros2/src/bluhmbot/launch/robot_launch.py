import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('bluhmbot')

    default_model_path = os.path.join(bringup_dir, 'description', 'bluhmbot_description.sdf')


    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    c920_config = os.path.join(
        bringup_dir, 'config', 'v4l2_camera.yaml')
    joystick_config = os.path.join(
        bringup_dir, 'config', 'joystick.yaml')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'depot.yaml'),
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
    declare_mock_usage = DeclareLaunchArgument(
        "use_mock_hardware", default_value="false", description="Start robot with mock hardware mirroring command to its states.",
    )

    # Declare arguments
    declared_arguments = [
        declare_namespace_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_log_level_cmd,
        declare_mock_usage,
        ]

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            default_model_path,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("bluhmbot"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

# When you're not using real hardware or a controller manager, and you just want to visualize or simulate joint movements in RViz use joint_state_publisher.
#    joint_state_publisher_node = Node(
#        package='joint_state_publisher',
#        executable='joint_state_publisher',
#        name='joint_state_publisher',
#        parameters=[{'robot_description': robot_description_content}],
#    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel_smoothed -r /diffbot_base_controller/odom:=/odom",
        ],
    )

    odom_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0.634804', '-1.86595', '0.15', '1.59969','0', '0', 'map', 'odom'],
    )

    status_led = Node(
            package='status_led',
            executable='status_led',
            name='status_led')

    teleop = Node(
            package='bluhmbot',
            executable='teleop.py',
            name='teleop')

    battery_management = Node(
            package='bluhmbot',
            executable='battery_management_node',
            name='battery_management_node')

    imu = Node(
            package='lsm6ds3',
            executable='lsm6ds3_node',
            name='lsm6ds3_node')

    utility_motors = Node(
            package='l298n',
            executable='l298n',
            name='l298n')

    camera = Node(
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
            ])
            
    localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
#                'namespace': namespace,
                'map': map_yaml_file,
#                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': params_file,
#                'use_composition': use_composition,
                'use_respawn': 'true',
                'container_name': 'nav2_container',
            }.items(),
        )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params, {'yaml_filename': map_yaml_file}],
        arguments=['--ros-args', '--log-level', log_level],
    )


    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )


    nodes = [
#        status_led,
        control_node,
#        imu,
        robot_state_publisher_node,
        robot_controller_spawner,
        odom_map_tf,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
#        utility_motors,
#        camera,
        map_server_node,
        localization_launch,
        teleop,
#        battery_management,
    ]

    return LaunchDescription(declared_arguments + nodes)
