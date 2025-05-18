#https://github.com/Milliways2/pi-gpio/blob/main/examples/setGPIO.c
#https://abyz.me.uk/rpi/pigpio/download.html

apt-get install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2-control-cmake
apt-get install -y unzip
cd ~/
curl -O -J -L -s https://github.com/joan2937/pigpio/archive/master.zip
unzip pigpio-master.zip
cd pigpio-master
make
make install
#rm -Rf pigpio-master*
#apt-get remove unzip




cd 
git clone https://github.com/ros-controls/ros2_control_demos   
cd ros2_control_demos/ros2_control_demo_description
colcon build; . install/setup.bash
cd ../example_10




