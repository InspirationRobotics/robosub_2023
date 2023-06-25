from robot_control import RobotControl
import time
import os

start = 114
rc = RobotControl()
time.sleep(1)
rc.setDepth(0.8) #setting depth
time.sleep(4)
rc.setHeading(106) # setting heading to start heading
time.sleep(2)
rc.forwardDist(7, 3) # auv moves forward 7 meters at power level 3
time.sleep(1)
rc.lateralUni(-1, 1.5) # auv moves laterally at power level 1 for 1.5 seconds
#time.sleep(2)
#t=0
#while t<2:
#    t=t+0.1
#    time.sleep(0.1)
#    rc.movement(throttle=0, forward=0, lateral=-1, yaw=0, pitch=0, roll=0)

rc.setDepth(0.4) # auv re-adjusts depth
time.sleep(5)
#deg = rc.return_compass()
for i in range(30): # auv yaws for 1.5 seconds at power level -1
  time.sleep(0.05)
  rc.movement(throttle=0, forward=0, lateral=0, yaw=-1, pitch=0, roll=0)
time.sleep(1)
rc.lateralUni(-1, 2) #auv moves laterally at power level 1 for 2 seconds
rc.forwardDist(12.3, 3) # auv moves forward at power level 3 for 12.3 meters
#print(deg)

#t=0
#while t<2:
 #   t=t+0.1
  #  time.sleep(0.1)
  #  rc.movement(throttle=0, forward=0, lateral=-2, yaw=0, pitch=0, roll=0)

deg = rc.return_compass()
rc.setHeading(deg+90) # auv turns 90
time.sleep(1)
rc.forwardDist(3, 3) #auv moves forward 3 meters at power level 3
time.sleep(1)
deg = rc.return_compass()
rc.setHeading(deg+93) # auv turns 93 degrees
rc.forwardDist(6.5, 2) # auv moves forward 6.5 meters at power level 2
time.sleep(2)
# t=0
# while t<2:
#     t=t+0.1
#     time.sleep(0.1)
#     rc.movement(throttle=0, forward=0, lateral=2, yaw=0, pitch=0, roll=0)
rc.lateralUni(2, 9.5) # auv moves laterally for 9.5 seconds at power level 2
time.sleep(2)
for i in range(45): # auv yaws for 2.25 seconds
  time.sleep(0.05)
  rc.movement(throttle=0, forward=0, lateral=0, yaw=1, pitch=0, roll=0)
time.sleep(2)
rc.setDepth(0.8) # auv re-adjusts depth to go under gate
rc.forwardDist(17, 3) #auv moves forward for 17 meters at power level 3
#rc.forwardDist(8, 3)
print("finished")
os.system("python3 /home/inspiration/auv/devices/disarm.py")
