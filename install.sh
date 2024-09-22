#/boot/config.txt
# hdmi_force_hotplug=1
# apt update
# apt upgrade
# apt-get install i2c-tools pip
# pip install smbus2

# Enable i2c
#. raspi-config nonint
#do_i2c 0
cp /root/robot/os/90-update.sh /lib/dhcpcd/dhcpcd-hooks/
# this should be a dynamic link
