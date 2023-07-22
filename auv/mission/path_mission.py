"""
Surfacing mission class
"""

import json
import os

import rospy
from std_msgs.msg import String

from auv.device import cvHandler
from auv.motion import robot_control
from auv.utils import disarm


class PathMission:
    cv_files = ["path_cv"]

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

        rospy.init_node("path_mission", anonymous=True)
        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cvHandler.CVHandler()

        # init the cv handlers
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        print("Path mission init")

    def callback(self, msg):
        """Callback for the cv_handler output, you can have multiple callback for multiple cv_handler"""
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

        print(f"Received data from {file_name}")

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """

        # move the sub up
        self.robot_control.setDepth(0.6)
        print(self.robot_control)
        while not rospy.is_shutdown():
            try:
                if not self.received:
                    continue

                for key in self.next_data.keys():
                    if key in self.data.keys():
                        self.data[key].update(self.next_data[key])
                    else:
                        self.data[key] = self.next_data[key]
                self.received = False
                self.next_data = {}

                if not "path_cv" in self.data.keys():
                    continue

                if self.data["path_cv"].get("end", False):
                    # idle the robot
                    self.robot_control.movement()
                    break

                print("starting movement")
                # get the lateral and forward values from the cv (if they exist)
                yaw = self.data["path_cv"].get("yaw", 0)
                forward = self.data["path_cv"].get("forward", 0)
                lateral = self.data["path_cv"].get("lateral", 0)
                print(yaw, forward, lateral)
                # direcly feed the cv output to the robot control
                self.robot_control.movement(yaw=yaw, forward=forward, lateral=lateral)

            except Exception as e:
                print(f"[ERROR] {e}")
                # idle the robot (just in case something went wrong)
                self.robot_control.movement()
                break

        print("Path mission finished")

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # idle the robot (just in case something went wrong)
        self.robot_control.movement()

        print("[INFO] Path mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.surfacing_mission"
    # You can also import it in a mission file outside of the package
    import time

    # Create a mission object with arguments
    mission = PathMission()

    # Run the mission
    mission.run()

    # cleanup
    mission.cleanup()
