import launch_ros.actions
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('bluhmbot')
    default_model_path = os.path.join(pkg_share, 'description', 'bluhmbot_description.sdf')
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

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
        
    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

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
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel_smoothed",
        ],
    )

    odom_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0.15', '0', '0', '0', 'map', 'odom'],
    )

    status_led = Node(
            package='status_led',
            executable='status_led',
            name='status_led')

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

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )


    nodes = [
        status_led,
        control_node,
        imu,
        robot_state_publisher_node,
        robot_controller_spawner,
        odom_map_tf,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        utility_motors,
        camera,
#        battery_management,
    ]

    return LaunchDescription(declared_arguments + nodes)
