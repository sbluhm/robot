set -ex
BRANCH=master
if [ -n "$1" ]
  then
    BRANCH=$1
fi

cd /usr/local/src
git clone https://github.com/berndporr/rpi_pwm.git ||:
cd rpi_pwm
git branch bluhmbot dc0d53899cfdfaee84cb2a789d584e985078e4a5
git switch bluhmbot
cmake .
make

