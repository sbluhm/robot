import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node'),
        launch_ros.actions.Node(
            package='zs_x11',
            executable='zs_x11',
            name='zs_x11'),
       launch_ros.actions.Node(
            package='motorcontroller',
            executable='motorcontroller',
            name='motorcontroller'),
  ])
