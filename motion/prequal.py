from robot_control import RobotControl
import time

rc = RobotControl()
time.sleep(1)
#rc.setDepth(0.4)


deg = rc.return_compass()
rc.setHeading(75)
rc.forwardDist(4.5, 3)
rc.forwardDist(0.5, 1)
rc.setHeading(165)
rc.forwardDist(1.2, 1)
rc.setHeading(255)
rc.forwardDist(0.5, 1)
rc.forwardDist(4.5, 3)
print("finished")
