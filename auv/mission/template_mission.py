"""
Template file to create a mission class
"""

# import what you need from within the package

import json
import logging

import rospy
from std_msgs.msg import String

from auv.device import cvHandler
from auv.motion import robot_control

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class TemplateMission:
    cv_files = ["template_cv"]

    def __init__(self, **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dict containing the settings you give to the mission
        """
        self.config = config
        self.data = {}  # dict to store the data from the cv handlers
        self.next_data = {}  # dict to store the data from the cv handlers
        self.received = False

        rospy.init_node("template_mission", anonymous=True)
        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cvHandler.CVHandler()

        # init the cv handlers
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        logger.info("Template mission init")

    def callback(self, msg):
        """Callback for the cv_handler output, you can have multiple callback for multiple cv_handler"""
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.data[file_name] = data
        self.next_data[file_name] = data
        self.received = True

        logger.debug("Received data from {}".format(file_name))

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """

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

            # TODO: do something with the data

            # here is an example of how to set a target
            self.cv_handler.set_target("template_cv", "albedo")

            break  # TODO: remove this line when making your mission

        logger.info("Template mission run")

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # idle the robot
        self.robot_control.movement()
        logger.info("Template mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package
    import time

    logging.basicConfig(level=logging.DEBUG)

    # Create a mission object with arguments
    mission = TemplateMission()

    # Run the mission
    mission.run()
    time.sleep(2)
    mission.cleanup()
