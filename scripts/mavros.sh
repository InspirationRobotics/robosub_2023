#!/bin/bash

PRODUCT=$(sudo lshw -json | jq '.product') || PRODUCT=$(sudo lshw -json | jq '.[].product')

if [[ $PRODUCT == *"Xavier"* ]]; then
  echo "Detected $PRODUCT setting to Xavier init"
  DISTRO="noetic"
fi
if [[ $PRODUCT == *"Nano"* ]]; then
  echo "Detected $PRODUCT setting to Nano init"
  DISTRO="melodic"
fi

OUTPUT=$(/usr/bin/python3 /home/inspiration/auv/scripts/deviceHelper.py)

echo "Found pixhawk on "${OUTPUT}
screen -dmS mavros bash -c "source /opt/ros/$DISTRO/setup.bash && sudo chmod 666 $OUTPUT && roslaunch mavros apm.launch fcu_url:=$OUTPUT"
echo "Done"
