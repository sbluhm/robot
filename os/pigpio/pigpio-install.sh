#https://abyz.me.uk/rpi/pigpio/download.html
#https://github.com/joan2937/pigpio

ref=c33738a320a3e28824af7807edafda440952c05d
cd /usr/local/src
curl -O -J -L -s https://github.com/joan2937/pigpio/archive/${ref}.tar.gz
tar xf pigpio-${ref}.tar.gz
ln -s pigpio-${ref} pigpio
cd pigpio
curl -O -J -L -s https://github.com/sbluhm/robot/raw/refs/heads/master/os/pigpio/ISRfix613.patch
patch < ISRfix613.patch
make
make install
