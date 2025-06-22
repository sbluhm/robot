set -e
BRANCH=master
if [ -n "$1" ]
  then
    BRANCH=$1
fi

cd /usr/local/src
git clone https://github.com/berndporr/rpi_pwm.git ||:
cd rpi_pwm
curl -O -J -L -s https://github.com/sbluhm/robot/raw/refs/heads/${BRANCH}/os/rpi_pwm/chip.patch
patch < chip.patch
cmake .
make

