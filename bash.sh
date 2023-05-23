gnome-terminal -- bash -c "ssh inspiration@jet-nano01.local; sudo chmod 777 /dev/ttyACM0; roslaunch mavros apm.launch"

# Open a new terminal window and execute the second set of commands
gnome-terminal -- bash -c "ssh inspiration@jet-nano01.local; rosrun mavros mavparam set SYSID_MYGCS 1; cd auv/devices/; python3 pix-standaloneOld.py"

# Open a new terminal window and execute the third set of commands
gnome-terminal -- bash -c "ssh inspiration@jet-nano01.local; cd auv/mission/; python3 square.py"