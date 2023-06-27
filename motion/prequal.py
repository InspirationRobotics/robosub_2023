from robot_control import RobotControl
import time
import os

start = 114
rc = RobotControl()
time.sleep(1)
rc.setDepth(0.8) #setting depth
time.sleep(4)
rc.forwardDist(19.3, 3)
time.sleep(2)

rc.lateralUni(2, 5)
deg = rc.return_compass()
rc.setHeading(deg+180) 
rc.forwardDist(6.5, 2)
time.sleep(2)

rc.lateralUni(2, 3) 
time.sleep(2)

rc.forwardDist(14, 3) #auv moves forward for 17 meters at power level 3
print("finished")
os.system("python3 /home/inspiration/auv/devices/disarm.py")
