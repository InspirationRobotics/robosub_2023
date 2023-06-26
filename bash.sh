#!/bin/bash

#screen -dmS mavros bash -c 'sudo chmod 666 /dev/ttyACM0 ; roslaunch mavros apm.launch'
#sleep 10
#screen -dmS cams bash -c '/usr/bin/python /home/inspiration/auv/camsVersatile.py'
sleep 40 #two minutes for tether disconnect
screen -dmS pix_standalone bash -c '/usr/bin/python3 /home/inspiration/auv/devices/pix_standalone.py'
/usr/bin/python3 /home/inspiration/auv/devices/statusLed.py redOn
sleep 25 #60
screen -dmS prequal bash -c '/usr/bin/python3 /home/inspiration/auv/motion/prequal.py'