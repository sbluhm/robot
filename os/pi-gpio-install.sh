apg-get install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
apt-get install unzip
cd /tmp
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
