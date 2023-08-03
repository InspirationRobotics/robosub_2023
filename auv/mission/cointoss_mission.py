"""
COINTOSS MISSION:
turns to the desired heading
"""

import time
import json

import rospy
from std_msgs.msg import String

from ..device import cvHandler
from ..motion import robot_control


class CoinTossMission:
    cv_files = []

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dict containing the settings you give to the mission
        """
        self.config = config

        self.robot_control = robot_control.RobotControl()

        print("[INFO] Coin Toss mission init")

    def run(self, heading):
        # the coin toss mission takes two parameters: heading and depth, which it will set accordingly

        print("[INFO] Coin Toss")
        time.sleep(1)
        self.robot_control.set_depth(0.65)  # setting depth, robot decends
        self.robot_control.setHeadingOld(heading)

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        """

        # idle the robot
        self.robot_control.movement()
        print("[INFO] Template mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package

    rospy.init_node("coin_toss", anonymous=True)
    # Create a mission object with arguments
    mission = CoinTossMission()

    # Run the mission
    mission.run(218)
    time.sleep(2)
    mission.cleanup()
