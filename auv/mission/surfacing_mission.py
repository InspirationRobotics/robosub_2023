"""
Surfacing mission class
"""

import json
import math
import os

import numpy as np

import rospy
from std_msgs.msg import String

from ..device import cvHandler
from ..motion import robot_control
from ..utils import disarm


class SurfacingMission:
    cv_files = ["surfacing_cv"]

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

        rospy.init_node("surfacing_mission", anonymous=True)
        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cvHandler.CVHandler(**self.config)

        # init the cv handlers
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.surfacing_sensitivity = self.config.get("surfacing_sensitivity", 0.5)
        self.proportional_gain = self.config.get("surfacing_proportional_gain", 4.0)
        print("[INFO] Surfacing mission init")

    def callback(self, msg):
        """Callback for the cv_handler output, you can have multiple callback for multiple cv_handler"""
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

        print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """

        # move the sub up
        self.robot_control.set_depth(0.5)

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

                if not "surfacing_cv" in self.data.keys():
                    continue

                if self.data["surfacing_cv"].get("end", None):
                    # idle the robot
                    print("ending True")
                    self.robot_control.movement()
                    break

                # get the lateral and forward values from the cv (if they exist)
                lateral = self.data["surfacing_cv"].get("lateral", 0)
                forward = self.data["surfacing_cv"].get("forward", 0)
                
                print(f"[DEBUG] lateral: {lateral}, forward: {forward}")
                self.robot_control.movement(lateral=lateral, forward=forward)

            except Exception as e:
                print(f"[ERROR] {e}")
                # idle the robot (just in case something went wrong)
                self.robot_control.movement()
                break

        print("[INFO] Template mission finished")

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # idle the robot (just in case something went wrong)
        self.robot_control.movement()
        print("[INFO] Template mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.surfacing_mission"
    # You can also import it in a mission file outside of the package

    from auv.utils import deviceHelper

    from auv.utils import deviceHelper

    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = SurfacingMission(**config)

    # Run the mission
    mission.run()

    # cleanup
    mission.cleanup()
