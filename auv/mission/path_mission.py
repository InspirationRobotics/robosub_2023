"""
For completing the Follow the Path task(s)
"""

import json
import os

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For controlling the motion of the sub
from ..utils import disarm


class PathMission:
    """
    Class to handle the running of the path mission
    """
    cv_files = ["path_cv"] # CV script to run

    def __init__(self, **config):
        """
        Initialize the PathMission class

        Args:
            config: Mission-specific keyword arguments to run the mission
        """
        self.config = config
        self.data = {}  # Dictionary to store the output from the CV handler(s)
        self.next_data = {}  #  Dictionary to store the most updated data from the CV handler(s) -- this will be merged with self.data
        self.received = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Initialize the CV handler
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        print("Path mission init")

    def callback(self, msg):
        """    
        Method to callback the cv_handler output. Multiple callbacks are possible for multiple cv_handlers. Converts the message data to a JSON format.
        
        Args:
            msg: Message data from the particular ROS topic. This is the dictionary of motion values that tells the sub how to move, and/or the visualized frame.
        """
        file_name = msg._connection_header["topic"].split("/")[-1] # Get the file name from the topic name
        data = json.loads(msg.data) # Convert the data to JSON
        self.next_data[file_name] = data
        self.received = True

        print(f"Received data from {file_name}")

    def run(self):
        """
        Run the follow the path mission
        """

        # Move the sub up (to 0.6 m)
        self.robot_control.set_depth(0.6)

        while not rospy.is_shutdown():
            try:
                if not self.received:
                    continue

                # Put the CV_handler data in self.data (with the correct key), and wipe next_data to receive the new inputs from the CV handler.
                for key in self.next_data.keys():
                    if key in self.data.keys():
                        self.data[key].update(self.next_data[key])
                    else:
                        self.data[key] = self.next_data[key]
                self.received = False
                self.next_data = {}

                if not "path_cv" in self.data.keys():
                    continue

                # Idle the robot if the mission has ended
                if self.data["path_cv"].get("end", False):
                    self.robot_control.movement()
                    break

                print("starting movement")

                # Get the lateral and forward values from the CV handler output (if they exist)
                yaw = self.data["path_cv"].get("yaw", 0)
                forward = self.data["path_cv"].get("forward", 0)
                lateral = self.data["path_cv"].get("lateral", 0)
                print(yaw, forward, lateral)

                # Directly feed the motion command values to the sub (technically this is not quite accurate, but it works)
                self.robot_control.movement(yaw=yaw, forward=forward, lateral=lateral)

            except Exception as e:
                print(f"[ERROR] {e}")
                # Idle the robot (just in case something went wrong)
                self.robot_control.movement()
                break

        print("Path mission finished")

    def cleanup(self):
        """
        Clean up the Follow the Path task by stopping the CV scripts and idling the sub
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name) # Stop the CV scripts

        # Idle the robot (just in case something went wrong)
        self.robot_control.movement()

        print("[INFO] Path mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.path_mission"
    # You can also import it in a mission file outside of the package

    import time
    from auv.utils import deviceHelper # Get the configuration of the devices on the sub

    rospy.init_node("path_mission", anonymous=True)

    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = PathMission(**config)

    # Run the mission
    mission.run()

    # Cleanup
    mission.cleanup()
