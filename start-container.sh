# Dummy lsm6ds3
if [[ `uname -m` == "x86_64" ]]; then 
sudo modprobe i2c-dev
sudo modprobe i2c-stub chip_addr=0x69
# i2cdetect -l
fi

sudo docker run -it --net=host --hostname=ros2-$(hostname) --privileged ros_docker
