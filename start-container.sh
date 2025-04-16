# Dummy lsm6ds3
if [[ `uname -m` == "x86_64" ]]; then 
sudo modprobe i2c-dev
sudo modprobe i2c-stub chip_addr=0x69
# i2cdetect -l
  sudo docker ps | grep ros_docker
  if [ $? -gt 0 ]; then
    echo "Starting a new container..."
    sudo docker run -it --net=host --privileged --env="DISPLAY=$DISPLAY" --volume="${XAUTHORITY}:/root/.Xauthority" --hostname=ros2-$(hostname) ros_docker
  else
    echo "Connecting to existing container..."
    sudo docker exec -it `sudo docker ps | grep ros_docker | sed 's/ .*//'` bash
  fi
else
  sudo docker run -it --net=host --hostname=ros2-$(hostname) --privileged ros_docker
fi
