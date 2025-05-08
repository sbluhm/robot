# ROS_DISTRO is set in Dockerfile as ENV
cd /root/robot && git pull --quiet
source /opt/ros/$ROS_DISTRO/setup.bash && cd /root/robot/ros2 && colcon build --symlink-install
