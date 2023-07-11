from auv.motion.robot_control import RobotControl
import time
import os

start = 331
rc = RobotControl()
time.sleep(1)
rc.setDepth(0.65) #setting depth
time.sleep(4)
rc.setHeading(start) 
rc.forwardDist(10, 2)
time.sleep(2)
#rc.lateralUni(-2, 3)
rc.forwardDist(4, 2)
time.sleep(2)
rc.lateralUni(-1, 3)
time.sleep(2)
rc.forwardDist(3.2, 2)
rc.lateralUni(2, 4)
time.sleep(2)
rc.setHeading(start+185) 
rc.forwardDist(9,2)
time.sleep(1)
rc.lateralUni(1, 8)
# startTime = time.time()
# while(time.time()-startTime<2):
#     rc.movement(throttle=0, forward=0, lateral=0, yaw=1, pitch=0, roll=0)
rc.forwardDist(7,2)
os.system("python3 /home/inspiration/auv/devices/disarm.py")
exit(1)
time.sleep(2)
rc.lateralUni(2, 5)
#deg = rc.return_compass()
rc.setHeading(start+170) 
rc.forwardDist(6.5, 2)
time.sleep(2)

rc.lateralUni(2, 3) 
time.sleep(2)

rc.forwardDist(14, 2) #auv moves forward for 17 meters at power level 3
print("finished")
os.system("python3 /home/inspiration/auv/devices/disarm.py")
