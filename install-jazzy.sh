# Installing ROS
ROS_DISTRO=jazzy
UBUNTU_DISTRO=noble

# Only works on 64bit OS (Raspberry OS 64bit)
if [[ `uname -m` != "x86_64" ]]; then
  echo "Update OS"
  sudo apt -y autoremove
  sudo apt-get -y update

  echo "Set locale"
  ## Set locale
  if [[ $(locale | grep UTF-8) ]]; then
  sudo apt-get -y install locales
  locale-gen de_DE de_DE.UTF-8
  update-locale LC_ALL=de_DE.UTF-8 LANG=de_DE.UTF-8
  export LANG=de_DE.UTF-8
  fi


  # Install docker
  echo "Remove Docker"
  for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do sudo apt-get -y remove $pkg; done
  echo "Install Docker"
  sudo apt-get -y install ca-certificates curl
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/raspbian/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt-get -y install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

  # Setup Docker container
  echo "Install Git"
  sudo apt-get -y install git
fi

echo "Updating Docker Container"
cd /tmp 
git clone https://github.com/osrf/docker_images/
#git submodule init
#git submodule update
if [[ `uname -m` == "x86_64" ]]; then
  cd docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/desktop-full
else
  cd docker_images/ros/${ROS_DISTRO}/ubuntu/${UBUNTU_DISTRO}/perception
fi
git checkout Dockerfile

# Dev environment:
# ros-humble-robot-localization

# ggfs wird ros-humble-image-transport-plugins benötigt
cat >> Dockerfile << EOF
SHELL ["/bin/bash", "-c"]
RUN apt-get -y update && apt-get -y upgrade && apt-get install -y --no-install-recommends \
    vim \
    ros-${ROS_DISTRO}-joy \
    v4l-utils \
    ros-${ROS_DISTRO}-v4l2-camera \
    python3-smbus2
RUN apt-get install -y --no-install-recommends python3-rpi.gpio ||:
RUN echo "export ROS_DOMAIN_ID=10" >> /root/.bashrc
RUN sed -i 's/exec/source "\/root\/robot\/ros2\/install\/setup.bash" --\nexec/' /ros_entrypoint.sh
RUN ln -s /root/robot/os/update.sh /update
RUN ln -s /root/robot/os/start.sh /start
RUN git clone https://github.com/sbluhm/robot /root/robot && echo $(date)
RUN /update
EOF

# Install additional packages on dev machine for navigation simulation
if [[ `uname -m` == "x86_64" ]]; then
cat >> Dockerfile << EOF
RUN mkdir -p  /lib/python3.10/RPi; cp /root/robot/os/RPi/* /lib/python3.10/RPi
RUN apt-get install -y --no-install-recommends \
    ros-dev-tools \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-localization
EOF
fi

# Don't clear apt cache
#RUN rm -rf /var/lib/apt/lists/*

# Delete the build cache before building to force OS update
#sudo docker builder prune --all
sudo docker build -t ros_docker_${ROS_DISTRO} .

# Start container
#sudo docker run -it --net=host --hostname=ros2-$(hostname) --privileged ros_docker
#sudo docker exec -it `sudo docker ps | grep ros_docker | sed 's/ .*//'` bash


# Joypad
#apt -y install ros-humble-joy
## https://index.ros.org/p/joy/github-ros-drivers-joystick_drivers/#humble
#export ROS_DOMAIN_ID=10
#ros2 run joy joy_enumerate_devices # show devices
#ros2 run joy joy_node
#ros2 topic echo /joy # test joystick
#sudo docker run -it --net=host --hostname=ros2-$(hostname) --privileged ros_docker ros2 run joy joy_node

# PS4 controller
# sudo bluetoothctl
#agent on
#discoverable on
#pairable on
#default-agent
#scan on
#connect CONTROLLER_MAC_ADDRESS
#trust CONTROLLER_MAC_ADDRESS
# systemctl enable --now bluetooth 


# Start Node from host
#sudo docker run  --net=host --hostname=ros2-$(hostname) --privileged -it ros_docker ros2 launch robot_launcher robot_launch.py



#/boot/config.txt
# hdmi_force_hotplug=1
# apt-get -y upgrade
# apt-get -y install i2c-tools pip
# pip install smbus2
# sudo dnf install i2c-tools

# Enable i2c
#. raspi-config nonint
#do_i2c 0

# Don't run this on Fedora
if [[ `uname -m` != "x86_64" ]]; then
  echo "Installing service"
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
fi

# Video
#pip install python3-opencv
# Test devices on Pi
# v4l2-ctl --list-devices
# Stream latency free
# apt-get -y install v4l-utils ffmpeg netcat-openbsd
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


#if which udevadm > /dev/null; then
#  set +e # Disable exit on error
#  udevadm control --reload-rules
#  service udev restart
#  udevadm trigger
#  set -e # Re-enable exit on error
#fi 
