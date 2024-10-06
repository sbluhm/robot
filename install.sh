# Installing ROS
# Only works on 64bit OS (Raspberry OS 64bit)

## Set locale
if [[ $(locale | grep UTF-8) ]]; then
apt update && sudo apt install locales
locale-gen de_DE de_DE.UTF-8
update-locale LC_ALL=de_DE.UTF-8 LANG=de_DE.UTF-8
export LANG=de_DE.UTF-8
fi


# Install docker
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do sudo apt-get remove $pkg; done
sudp apt update
sudo apt upgrade
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/raspbian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Setup Docker container
sudo apt-get install git
cd 
git clone https://github.com/osrf/docker_images/
cd docker_images/ros/humble/ubuntu/jammy/perception
docker build -t ros_docker .

# Start container
#sudo docker run -it --net=host ros_docker
sudo docker run -it --net=host --privileged  ros_docker
apt update
#apt upgrade

# Joypad
apt -y install ros-humble-joy
# https://index.ros.org/p/joy/github-ros-drivers-joystick_drivers/#humble
export ROS_DOMAIN_ID=10
ros2 run joy joy_enumerate_devices # show devices
ros2 run joy joy_node
#ros2 topic echo /joy # test joystick

# Start Node from host
docker run -it --rm osrf/ros:foxy-desktop ros2 run demo_nodes_cpp talker

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
chmod a+x gamepad.py
chmod a+x install.sh
# this should be a dynamic link

# Video
#pip install python3-opencv
