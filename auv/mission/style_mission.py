"""
STYLE MISSION:
Turns 720 degrees and then returns to the initial heading
"""

import time
import json

import rospy
from std_msgs.msg import String

from ..device import cvHandler
from ..motion import robot_control


class StyleMission:
    cv_files = []

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dict containing the settings you give to the mission
        """
        self.config = config

        self.robot_control = robot_control.RobotControl()

        print("[INFO] style_mission init")

    def run(self):
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

        print("[INFO] Style mission run")

        self.robot_control.movement(yaw=2)  # turn
        time.sleep(8)  # keep turning
        self.robot_control.movement(yaw=0)

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
    
    rospy.init_node("style_mission", anonymous=True)
    # Create a mission object with arguments
    mission = StyleMission()

    # Run the mission
    mission.run()
    time.sleep(2)
    mission.cleanup()
