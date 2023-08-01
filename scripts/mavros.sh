#!/bin/bash

PRODUCT=$(sudo lshw -json | jq '.product') || PRODUCT=$(sudo lshw -json | jq '.[].product')

if [[ $PRODUCT == *"Xavier"* ]]; then
  echo "Detected $PRODUCT setting to Xavier init"
  POLULU=$(/usr/bin/python3 /home/inspiration/auv/auv/utils/deviceHelper.py polulu)
  echo "Found Polulu Servo driver at $POLULU"
  screen -dmS polulu bash -c "bash /home/inspiration/auv/maestro-linux/clearPoluluErrors.sh $POLULU"
  DISTRO="noetic"
fi
if [[ $PRODUCT == *"Nano"* ]]; then
  echo "Detected $PRODUCT setting to Nano init"
  DISTRO="melodic"
fi

OUTPUT=$(/usr/bin/python3 /home/inspiration/auv/auv/utils/deviceHelper.py)

echo "Found pixhawk on "${OUTPUT}
screen -dmS roscore bash -c "source /opt/ros/$DISTRO/setup.bash ; roscore"
screen -dmS mavros bash -c "source /opt/ros/$DISTRO/setup.bash ; sleep 5 ; roslaunch --wait mavros apm.launch fcu_url:=$OUTPUT"
echo "Done"
# TODO: fix mavros