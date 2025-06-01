F2C_VERSION=v1.2.1-devel
git clone -b $F2C_VERSION https://github.com/Fields2Cover/Fields2Cover /root/ros2/Fields2Cover --quiet
cd /root/ros2
git clone https://github.com/open-navigation/opennav_coverage
colcon build; . install/setup.bash
