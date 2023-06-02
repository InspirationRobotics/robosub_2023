from robot_control import RobotControl
import time

rc = RobotControl()
time.sleep(1)
rc.setDepth(0.65)


deg = rc.return_compass()
rc.setHeading(75)
rc.forwardDist(5.5, 3)

t=0
while t<2:
    t=t+0.1
    time.sleep(0.1)
    rc.movement(throttle=0, forward=0, lateral=-2, yaw=0, pitch=0, roll=0)

rc.forwardDist(1, 3)
rc.forwardDist(1, 1)
rc.setHeading(165)
rc.forwardDist(2, 1)
rc.setHeading(250)
rc.forwardDist(1, 1)
rc.forwardDist(7, 3)
print("finished")
