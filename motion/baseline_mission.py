from robot_control import RobotControl
import time

rc = RobotControl()
rc.movement(throttle=-5, forward=0, lateral=0, yaw=0, pitch=0, roll=0, t=5)
time.sleep(10)
#rc.movement(throttle=-18.75, forward=3, lateral=0, yaw=0, pitch=0, roll=0, t=4)
#rc.movement(throttle=0, forward=0, lateral=2, yaw=0, pitch=0, roll=0, t=3)
#rc.movement(throttle=0, forward=-2, lateral=0, yaw=0, pitch=0, roll=0, t=3)
#rc.movement(throttle=0, forward=0, lateral=-2, yaw=0, pitch=0, roll=0, t=3)
print("finished")
