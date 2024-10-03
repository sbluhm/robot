# Installing ROS

## Set locale
if [[ $(locale | grep UTF-8) ]]; then
apt update && sudo apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
fi

## Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

## Install
sudo apt update
#sudo apt upgrade -y
#sudo apt install ros-humble-ros-base
#sudo apt install ros-dev-tools

#/boot/config.txt
# hdmi_force_hotplug=1
# apt update
# apt upgrade
# apt-get install i2c-tools pip
# pip install smbus2

# Enable i2c
#. raspi-config nonint
#do_i2c 0
#cp /root/robot/os/90-update.sh /lib/dhcpcd/dhcpcd-hooks/
cp /root/robot/os/robot-update.service /etc/systemd/system
cp /root/robot/os/robot.service /etc/systemd/system
systemctl daemon-reload
systemctl enable robot-update.service
systemctl enable robot.service
systemctl start robot.service
chmod a+x robot.py
chmod a+x install.sh
# this should be a dynamic link

# Video
#pip install python3-opencv
