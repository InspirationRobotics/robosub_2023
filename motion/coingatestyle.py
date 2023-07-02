from robot_control import RobotControl
import time
import os

# For july 3rd pool test
# Coin toss, gate, and style points with working computer vision.   

start = 331 # this is the heading the robot will align to at the beginning of the program
rc = RobotControl()

# coin toss
time.sleep(1)
rc.setDepth(0.65) #setting depth
time.sleep(4)
rc.setHeading(start) 

# move towards the gate
rc.forwardDist(2, 2) 
# for the monday pool test use (5,2) goes forward for 5 seconds at power of 2 which is about 
time.sleep(2)

# insert the cv code here 

# move through the gate
rc.forwardDist(2, 2) 

# style points, yaw 720 degrees
rc.setHeading(start+720) 

os.system("python3 /home/inspiration/auv/devices/disarm.py")
exit(1)

print("finished")
