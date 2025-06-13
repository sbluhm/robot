dtc -I dts -O dtb -o pwm-pi5.dtbo pwm-pi5-overlay.dts
cp pwm-pi5.dtbo /boot/firmware/overlays/
echo "dtoverlay=pwm-pi5" >> /boot/firmware/config.txt
