from robot_control import RobotControl
import time
import os
from servos import *

# For july 3rd pool test, written on 7/1/2023 and 7/2/2023
# Coin toss, gate, and style points with working computer vision, autonomously fire torpedoes and drop the markers

start = 331 # this is the heading the robot will align to at the beginning of the program, need to update when go to the pool
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

# add in torpedo shooter - shoot one, pause, change depth, pause, shoot the second
torpedoLauncher(1) # shoots first torpedo
time.sleep(1)
torpedoLauncher(2) # shoots second torpedo
time.sleep(1)
# move forward

# drop both markers
dropper(1) # drops first marker
time.sleep(1)
dropper(2) # drops second marker
time.sleep(1)
os.system("python3 /home/inspiration/auv/devices/disarm.py")
exit(1)

print("finished")