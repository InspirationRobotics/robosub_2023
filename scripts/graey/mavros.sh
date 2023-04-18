sudo chmod 666 $1
screen -dm roscore
sleep 5
screen -dm 'roslaunch mavros apm.launch'
sleep 5
rosrun mavros mavparam set SYSID_MYGCS 1
