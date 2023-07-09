from robot_control import RobotControl
import time
import os
from motion import servos as servo

# For july 3rd pool test, written on 7/1/2023 and 7/2/2023
# Coin toss, gate, and style points with working computer vision, autonomously fire torpedoes and drop the markers
# Note that this was based on the most recent prequalification code which was for Graey not Onyx

gate = 331 # this is the heading the robot will align to at the beginning of the program, need to update when go to the pool
torpedo1 = True # default torpedoes values are either loaded or unloaded
torpedo2 = True
marker1 = True # default markers values are either loaded or unloaded
marker2 = True
rc = RobotControl()

# coin toss
time.sleep(1)
rc.setDepth(0.65) #setting depth
time.sleep(4)
rc.setHeading(gate) 

# move towards the gate
rc.forwardDist(5, 2) 
# for the monday pool test use (5,2) goes forward for 5 seconds at power of 2 which is about 13 meters
time.sleep(2)

# insert the cv code here 

# move through the gate
rc.forwardDist(2, 2) 

# style points, yaw 720 degrees
rc.setHeading(gate+720) 

# torpedo shooter - shoot one, pause, change depth, pause, shoot the second
rc.forwardDist(2, 2) 
if torpedo1 and torpedo2 == True:
  servo.torpedoLauncher(1) # shoots first torpedo
  torpedo1 = False  
  time.sleep(1)
  rc.setDepth(0.9) # change depth, moving down .5 meters
  servo.torpedoLauncher(2) # shoots second torpedo
  torpedo2 = False
  time.sleep(1)
else:
  rc.forwardDist(1, 2)  


if marker1 and marker2 == True:

    # drop both markers
    servo.dropper(1) # drops first marker
    time.sleep(1)
    servo.dropper(2) # drops second marker
    time.sleep(1)

else:
    time.sleep(4)
os.system("python3 /home/inspiration/auv/devices/disarm.py")
exit(1)

print("finished")
