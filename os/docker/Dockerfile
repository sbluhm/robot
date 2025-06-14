ENV ROS_DISTRO="jazzy"
ENV GIT_CURRENT_BRANCH="route-planning"

FROM ros:jazzy-ros-base-noble
RUN export GIT_CURRENT_BRANCH="route-planning"
RUN export ROS_DISTRO="jazzy"
# install ros2 packages
RUN apt-get update && apt-get -y upgrade

RUN if [ "$(uname -m)" = "x86_64" ]; then \
    apt-get install -y --no-install-recommends \
    ros-jazzy-desktop-full=0.11.0-1*; \
    else \
    apt-get install -y --no-install-recommends \
    ros-jazzy-perception=0.11.0-1*; \
    fi

RUN apt-get install -y --no-install-recommends     vim     ros-jazzy-joy ros-jazzy-teleop-twist-joy    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2-control-cmake     v4l-utils     ros-jazzy-v4l2-camera     ros-jazzy-navigation2     ros-jazzy-xacro     ros-jazzy-joint-state-publisher     ros-jazzy-robot-localization     ros-jazzy-sdformat-urdf     python3-smbus2

RUN if [ "$(uname -m)" != "x86_64" ]; then apt-get install -y --no-install-recommends python3-rpi-lgpio; else ln -s /root/robot/os/RPi /lib/python3.12/RPi; fi
RUN echo "export ROS_DOMAIN_ID=10" >> /root/.bashrc
RUN echo 'alias cdb="cd ~/robot/ros2/src/bluhmbot"' >> /root/.bashrc
RUN echo 'source "/opt/ros/jazzy/setup.bash" --' >> /root/.bashrc
RUN echo 'source "/root/robot/ros2/install/setup.bash" --' >> /root/.bashrc
RUN sed -i 's/exec/source "\/root\/robot\/ros2\/install\/setup.bash" --\nexport ROS_DOMAIN_ID=10\nexec/' /ros_entrypoint.sh
RUN ln -s /root/robot/os/update.sh /update
RUN if [ "$(uname -m)" = "x86_64" ]; then ln -sf /root/robot/os/start-mock.sh /start; else ln -s /root/robot/os/start.sh /start; fi
RUN ln -s /root/robot/os/nav2.sh /nav2
RUN ln -s /root/robot/os/rviz.sh /rviz
ARG ROS_DISTRO="jazzy"
ARG GIT_CURRENT_BRANCH="route-planning"

RUN echo "currentbranch: ${GIT_CURRENT_BRANCH}"

RUN curl https://raw.githubusercontent.com/sbluhm/robot/refs/heads/$GIT_CURRENT_BRANCH/os/pigpio/pigpio-install.sh | bash
RUN source "/opt/ros/jazzy/setup.bash"; curl https://raw.githubusercontent.com/sbluhm/robot/refs/heads/$GIT_CURRENT_BRANCH/os/opennav_coverage/opennav_coverage-install.sh | bash
RUN if [ "$(uname -m)" = "x86_64" ]; then apt-get install -y --no-install-recommends \
    ros-dev-tools \
    ros-jazzy-nav2-loopback-sim ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-slam-toolbox ros-jazzy-turtlebot3-bringup ros-jazzy-slam-toolbox \
    ros-$ROS_DISTRO-turtlebot3-gazebo \
    ros-$ROS_DISTRO-nav2-minimal-tb3-sim ros-$ROS_DISTRO-nav2-bringup; \
    fi

RUN git clone -b $GIT_CURRENT_BRANCH  https://github.com/sbluhm/robot /root/robot && echo "$(date)"
RUN export ROS_DISTRO=jazzy
RUN /update

