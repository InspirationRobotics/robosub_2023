"""
COINTOSS MISSION:
turns to the desired heading
"""

import time
import json
import logging

import rospy
from std_msgs.msg import String

from auv.device import cvHandler
from auv.motion import robot_control

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class CoinTossMission:
    cv_files = []

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dict containing the settings you give to the mission
        """
        self.config = config

        rospy.init_node("coin_toss", anonymous=True)
        self.robot_control = robot_control.RobotControl()

        logger.info("Coin Toss mission init")

    def run(self, heading, depth):
        # the coin toss mission takes two parameters: heading and depth, which it will set accordingly

        while not rospy.is_shutdown():
            if not self.received:
                continue

            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

        logger.info("Coin Toss")

        time.sleep(1)
        self.robot_control.setDepth(depth)  # setting depth, robot decends
        time.sleep(5)  # wait
        self.robot_control.setHeading(heading)  # turning to the heading

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        """

        # idle the robot
        self.robot_control.movement()
        logger.info("Template mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package

    logging.basicConfig(level=logging.DEBUG)

    # Create a mission object with arguments
    mission = TemplateMission()

    # Run the mission
    mission.run()
    time.sleep(2)
    mission.cleanup()