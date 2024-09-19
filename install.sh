#/boot/config.txt
# hdmi_force_hotplug=1
dnf -y install python3-inputs
cp os/90-update.sh /lib/dhcpcd/dhcpcd-hooks/
