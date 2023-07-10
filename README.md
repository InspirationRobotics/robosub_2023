# robosub_2023
Team Inspiration's repository for the RoboSub 2023 competition. We are using ROS, python, and C++.

*Please use this repository only for source files*; do not upload binary files like images/videos/ROSBags.

**TO DO:**

write the .gitignore file

create the basic empty template programs 
- one for a hypothetical sensor stub, one mapper, a mission planner, and one for MAVROS (this will be made into a path follower)

verify integration using the template programs

camsVersatile

Colin Notes:
everythings in the Redesign branch

devices - most important folder in the repo
Pix_standalone- most important script in the repo, controlls most everything for graey, alt hold to maintain depth, establishes all of the topics, imu, compass, motor, battery (pix standalone) 

"bash.sh" startup script

subcribes all to all of the 

auv/devices/compass 
auv/devices/setDepth # PID control custom depth

Motion - second most important folder

"robot_control.py" 

# getting current heading of the compass, reads data from the pixahawk, mavros is the ros interface with the pixahawk
rostopic echo /mavros/global_position/compass_hdg

utilzie the motion folder, for the mission runs
