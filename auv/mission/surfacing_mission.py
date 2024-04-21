"""
Class for running the surface mission.
Moves to the correct position inside each chevron, and surfaces "in" each.
"""

import json
import math
import os

import numpy as np

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For controling the thrusters on the sub (to move it)
from ..utils import disarm


class SurfacingMission:
    cv_files = ["surfacing_cv"]

    def __init__(self, **config):
        """
        Initialize the Surfacing Mission class. 

        Args:
            config (dict): Configuration settings to run the mission
        """
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handler
        self.next_data = {}  # Dictionary to store the most updated data from the CV handler
        self.received = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Initialize the CV script
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        print("[INFO] Surfacing mission init")

    def callback(self, msg):
        """
        Callback function for obtaining output from the CV handler. Multiple callback functions can be used for multiple CV handlers. Converts the 
        message data to JSON format.

        Args:
            msg: Output from the CV handler. This will be a dictionary of motion values, possible the visualized frame, and potentially servo commands
        """
        file_name = msg._connection_header["topic"].split("/")[-1] # Get the CV file name from the topic name
        data = json.loads(msg.data) # Load the JSON data
        self.next_data[file_name] = data
        self.received = True

        print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Run the surfacing mission.
        """

        # Move the sub up
        self.robot_control.set_depth(0.65)

        while not rospy.is_shutdown():
            try:
                if not self.received:
                    continue

                # Merge self.next_data, which contains the updated CV handler output, with self.data, which contains the previous CV handler output.
                # self.next_data will be wiped so that it can be updated with the new CV handler output.
                for key in self.next_data.keys():
                    if key in self.data.keys():
                        self.data[key].update(self.next_data[key]) # Merge the data
                    else:
                        self.data[key] = self.next_data[key] # Update the keys if necessary
                self.received = False
                self.next_data = {} # Wipe self.next_data

                if not "surfacing_cv" in self.data.keys():
                    continue

                if self.data["surfacing_cv"].get("end", None):
                    # Idle the robot if the surfacing CV script says to end
                    self.robot_control.movement()
                    break

                # Get the lateral and forward values from the CV script (if they exist)
                lateral = self.data["surfacing_cv"].get("lateral", 0)
                forward = self.data["surfacing_cv"].get("forward", 0)
                
                print(f"[DEBUG] lateral: {lateral}, forward: {forward}")
                self.robot_control.movement(lateral=lateral, forward=forward)

            except Exception as e:
                print(f"[ERROR] {e}")
                # Idle the robot (just in case something went wrong)
                self.robot_control.movement()
                break

        print("[INFO] Template mission finished") # From copied code
        # disarm.disarm()

    def cleanup(self):
        """
        Clean up after the surfacing mission. Stops the CV script, and idles the robot.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name) # Stop the CV script

        # Idle the robot (just in case something went wrong)
        self.robot_control.movement()
        print("[INFO] Template mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
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

    rospy.init_node("surfacing_mission", anonymous=True)

    # Create a mission object with arguments
    mission = SurfacingMission(**config)

    # Run the mission
    mission.run()

    # Clean up the mission
    mission.cleanup()
