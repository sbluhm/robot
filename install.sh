# Installing ROS
# Only works on 64bit OS (Raspberry OS 64bit)

sudo apt autoremove

## Set locale
if [[ $(locale | grep UTF-8) ]]; then
apt update && sudo apt install locales
locale-gen de_DE de_DE.UTF-8
update-locale LC_ALL=de_DE.UTF-8 LANG=de_DE.UTF-8
export LANG=de_DE.UTF-8
fi


# Install docker
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do sudo apt-get remove $pkg; done
sudo apt update
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
cd /tmp 
git clone https://github.com/osrf/docker_images/
cd docker_images/ros/humble/ubuntu/jammy/perception


# ggfs wird ros-humble-image-transport-plugins benötigt
cat >> Dockerfile << EOF
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get -y upgrade && apt-get install -y --no-install-recommends \
    vim \
    ros-humble-joy \
    v4l-utils \
    ros-humble-v4l2-camera \
    python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/*

RUN echo "export ROS_DOMAIN_ID=10" >> /root/.bashrc
RUN git clone https://github.com/sbluhm/robot /root/robot
RUN source /opt/ros/humble/setup.bash && cd /root/robot/ros2 && colcon build
RUN sed -i 's/exec/source "\/root\/robot\/ros2\/install\/setup.bash" --\nexec/' /ros_entrypoint.sh 
RUN echo "ros2 launch robot_launcher robot_launch.py" > /start && chmod a+x /start
RUN echo "cd /root/robot && git pull" > /update && chmod a+x /update
RUN echo "source /opt/ros/humble/setup.bash && cd /root/robot/ros2 && colcon build" >> /update
EOF

# Delete the build cache before building
docker builder prune --all
docker build -t ros_docker .

# Start container
sudo docker run -it --net=host --hostname=ros2-$(hostname) --privileged  ros_docker

# Joypad
#apt -y install ros-humble-joy
## https://index.ros.org/p/joy/github-ros-drivers-joystick_drivers/#humble
#export ROS_DOMAIN_ID=10
#ros2 run joy joy_enumerate_devices # show devices
#ros2 run joy joy_node
#ros2 topic echo /joy # test joystick



# Start Node from host
sudo docker run  --net=host --hostname=ros2-$(hostname) --privileged -it ros_docker ros2 launch robot_launcher robot_launch.py



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
# Test devices on Pi
# v4l2-ctl --list-devices
# Stream latency free
# apt install v4l-utils ffmpeg netcat-openbsd
# ffmpeg -i /dev/video0 -codec copy - | nc -l 9999

# on fedora

#dnf install kernel-devel kernel-headers dkms v4l-utils
#git clone https://github.com/umlaeute/v4l2loopback.git
#cd v4l2loopback
#make
#sudo cp -R . /usr/src/v4l2loopback-1.1
#sudo dkms add -m v4l2loopback -v 1.1
#sudo dkms build -m v4l2loopback -v 1.1
#sudo dkms install -m v4l2loopback -v 1.1
# import dkms key into safeboot
#sudo mokutil --import /var/lib/dkms/mok.pub

#reboot now
#after reboot execute the following commands
#sudo depmod -a
#sudo modprobe v4l2loopback card_label="Camera" 
