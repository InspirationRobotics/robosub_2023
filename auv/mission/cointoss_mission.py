"""
Runs the cointoss mission.
Turns to the desired heading.
"""

import time
import json

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For controlling the thrusters of the sub


class CoinTossMission:
    """
    Class to run the Cointoss mission
    """
    cv_files = []

    def __init__(self, **config):
        """
        Initialize the CoinTossMission class

        Args:
            config (dict): Configuration settings to run the mission
        """
        self.config = config

        self.robot_control = robot_control.RobotControl()

        print("[INFO] Coin Toss mission init")

    def run(self, heading):
        """
        Run the cointoss mission.

        Args:
            heading: the desired heading to turn the sub to.
        """

        print("[INFO] Coin Toss")
        time.sleep(1)
        self.robot_control.set_depth(0.65)  # Robot descends to specified depth (0.65 m)
        self.robot_control.setHeadingOld(heading)

    def cleanup(self):
        """
        Clean up the cointoss mission. Idles the sub.
        """

        # Idle the robot
        self.robot_control.movement()
        print("[INFO] Template mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.cointoss_mission"
    # You can also import it in a mission file outside of the package

    rospy.init_node("coin_toss", anonymous=True)
    # Create a mission object with arguments
    mission = CoinTossMission()

    # Run the mission
    mission.run(218)
    time.sleep(2)
    mission.cleanup()
