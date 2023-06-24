from robot_control import RobotControl
import time
import os

rc = RobotControl()
time.sleep(1)
#rc.setDepth(0.5)



rc.forwardDist(10, 3)
deg = rc.return_compass()
print(deg)

#t=0
#while t<2:
 #   t=t+0.1
  #  time.sleep(0.1)
  #  rc.movement(throttle=0, forward=0, lateral=-2, yaw=0, pitch=0, roll=0)

rc.setHeading(deg+90)
rc.forwardDist(1.5, 1)
deg = rc.return_compass()
rc.setHeading(deg+90)
rc.forwardDist(3, 3)
t=0
while t<1.5:
    t=t+0.1
    time.sleep(0.1)
    rc.movement(throttle=0, forward=0, lateral=2, yaw=0, pitch=0, roll=0)

rc.forwardDist(5.5, 3)
print("finished")
os.system("python3 /home/inspiration/auv/devices/disarm.py")