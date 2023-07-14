"""
Template file to create a mission class
"""

# import what you need from within the package

import json
import logging

import rospy
from std_msgs.msg import String


class TemplateMission:
    def __init__(self, robot_control, **kwargs):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        kwargs is a dict containing the settings you give to the mission
        """
        self.rc = robot_control

        self.pub = rospy.Publisher("auv/cv_handler/cmd", String, queue_size=1)
        self.sub = rospy.Subscriber("auv/cv_handler/output", String, self.callback)
        logging.info("Template mission init")

    def callback(self, data):
        """Callback for the cv_handler output"""
        # TODO

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        Return True if everything went well, False otherwise (ideally)
        """
        logging.info("Template mission run")
        return True

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        Return True if everything went well, False otherwise (ideally)
        """
        logging.info("Template mission terminate")
        return True


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package

    # Create a mission object with arguments
    # (usually you should pass the robot_control object to the mission)
    mission = TemplateMission(None, arg1="value1", arg2="value2")

    # run the mission
    mission.run()
