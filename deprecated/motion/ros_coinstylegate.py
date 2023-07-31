#!/usr/bin/env python
import os
import time

import rospy
from robot_control import RobotControl
from std_msgs.msg import int32

from auv.motion.servo import Servo

gate = 331  # this is the heading the robot will align to at the beginning of the program, need to update when go to the pool
torpedo1 = True  # default torpedoes values are either loaded or unloaded
torpedo2 = True
marker1 = True  # default markers values are either loaded or unloaded
marker2 = True
rc = RobotControl()
servo = Servo()


def cv_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Data: %s", data.data)
    heading += data.data  # modify target heading based on cv output


def main():
    rospy.init_node("cv", anonymous=True)

    rospy.Subscriber("/auv/data/cv_heading", int32, cv_callback)

    # coin toss
    time.sleep(1)
    rc.setDepth(0.65)  # setting depth
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
    rc.setHeading(gate + 720)

    # torpedo shooter - shoot one, pause, change depth, pause, shoot the second
    rc.forwardDist(2, 2)
    if torpedo1 and torpedo2 == True:
        servo.torpedoLauncher(1)  # shoots first torpedo
        torpedo1 = False
        time.sleep(1)
        rc.setDepth(0.9)  # change depth, moving down .5 meters
        servo.torpedoLauncher(2)  # shoots second torpedo
        torpedo2 = False
        time.sleep(1)
    else:
        rc.forwardDist(1, 2)

    if marker1 and marker2 == True:
        # drop both markers
        servo.dropper(1)  # drops first marker
        time.sleep(1)
        servo.dropper(2)  # drops second marker
        time.sleep(1)

    else:
        time.sleep(4)
    os.system("python3 /home/inspiration/auv/devices/disarm.py")
    exit(1)

    print("finished")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    main()
