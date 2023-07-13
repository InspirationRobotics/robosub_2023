# Notes on getting started with writing your an autonomous mission

“Missions” are just programs that tell the AUV, on a high level, how and where to move according to what rules. Even moving in a square is considered a “mission.”

I assume you already know how publishers and subscribers in ROS work. There is a program, motion_handler.py (under motion/), that is listening on the /auv/motion/raw topic for an array of numbers that will tell the sub how to move. This program serves as a frontend for our flight controller that uses MAVLink, so if you want to actually control the motors, you should first run MAVROS[1] and then run motion_handler.py.

Each number is a pwm value, ranging from 1100 (negative) to 1900 (positive). A value of 1500 will result in no motion. There is usually no reason to go all the way to 1900. Rather than corresponding to individual motors, each pwm value in the array corresponds to a different motion--for example, the fifth(?) channel in the array corresponds to lateral motion. You can blend movements (correct me (this text file) if I am wrong) by inputting values for multiple channels.

In order to get the AUV to do something (move), create a new program under mission/, and then publish said array of integer values to the aforementioned topic. This should enable you to command the AUV to move in any way you want. You can create subscribers to topics that have sensor data and use that in tandem with motor commands to create intelligent behavior.

[1]
`roscore`
`roslaunch mavros apm.launch`
`rosrun mavros mavparam set SYSID_MYGCS 1`

Note: there is a script for this already on Graey which we’ll soon put on the Github: [insert the path to that here]
