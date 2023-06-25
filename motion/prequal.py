from robot_control import RobotControl
import time
import os

start = 114
rc = RobotControl()
time.sleep(1)
rc.setDepth(0.8)
time.sleep(4)
rc.setHeading(106)
time.sleep(2)
rc.forwardDist(7, 3)
time.sleep(1)
rc.lateralUni(-1, 1.5)
#time.sleep(2)
#t=0
#while t<2:
#    t=t+0.1
#    time.sleep(0.1)
#    rc.movement(throttle=0, forward=0, lateral=-1, yaw=0, pitch=0, roll=0)

rc.setDepth(0.4)
time.sleep(5)
#deg = rc.return_compass()
for i in range(30):
  time.sleep(0.05)
  rc.movement(throttle=0, forward=0, lateral=0, yaw=-1, pitch=0, roll=0)
time.sleep(1)
rc.lateralUni(-1, 2)
rc.forwardDist(12.3, 3)
#print(deg)

#t=0
#while t<2:
 #   t=t+0.1
  #  time.sleep(0.1)
  #  rc.movement(throttle=0, forward=0, lateral=-2, yaw=0, pitch=0, roll=0)

deg = rc.return_compass()
rc.setHeading(deg+90)
time.sleep(1)
rc.forwardDist(3, 3)
time.sleep(1)
deg = rc.return_compass()
rc.setHeading(deg+93)
rc.forwardDist(6.5, 2)
time.sleep(2)
# t=0
# while t<2:
#     t=t+0.1
#     time.sleep(0.1)
#     rc.movement(throttle=0, forward=0, lateral=2, yaw=0, pitch=0, roll=0)
rc.lateralUni(2, 9.5)
time.sleep(2)
for i in range(45):
  time.sleep(0.05)
  rc.movement(throttle=0, forward=0, lateral=0, yaw=1, pitch=0, roll=0)
time.sleep(2)
rc.setDepth(0.8)
rc.forwardDist(17, 3)
#rc.forwardDist(8, 3)
print("finished")
os.system("python3 /home/inspiration/auv/devices/disarm.py")
