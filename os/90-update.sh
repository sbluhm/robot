cd /root/robot
git stash
git pull
/root/robot/install.sh
echo "Starting gamepad" >> /var/log/robot.log
python robot.py >> /var/log/robot.log
echo "Gamepad stopped" >> /var/log/robot.log
