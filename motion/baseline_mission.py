from robot_control import RobotControl

rc = RobotControl()
rc.movement(throttle=0, forward=2, lateral=0, yaw=0, pitch=0, roll=0, t=2)
print("finished")
