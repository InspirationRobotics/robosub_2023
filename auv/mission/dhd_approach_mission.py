"""
Class in order to complete the DHD Approach mission.
Approach the DHD (the center of the Octagon) using a perception loop.
"""

import json

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For controlling the motion of the robot


class DHDApproachMission:
    cv_files = ["dhd_approach_cv"] # CV file to run

    def __init__(self, **config):
        """
        Initialize the DHD approach mission

        Args:
            config: Mission-specific parameters to run the mission
        """
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handlers
        self.next_data = {}  # Dictionary to store the newest data from the CV handlers -- this will be merged with self.data
        self.received = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Initialize the CV handlers
        # Dummy streams are used to input a video file for the CV script to run on rather than a true camera stream
        dummys = self.config.get("cv_dummy", [None] * len(self.cv_files))
        for file_name, dummy in zip(self.cv_files, dummys):
            self.cv_handler.start_cv(file_name, self.callback, dummy_camera=dummy)

        print("[INFO] dvl approach mission init")

    def callback(self, msg):
        """
        Callback for the cv_handler output. Converts the output to JSON format and puts it in self.next_data, the list that holds 
        the most updated CV output data.

        Args:
            msg: Data from the CV handler
        """
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        """
        Running the DHD approach mission.
        """

        while not rospy.is_shutdown():
            if not self.received:
                continue

            # Merge the data from self.next_data with self.data, and wipe self.next_data so it can take the new, more updated data from
            # the CV handler.
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key]) # Merge the data with the corresponding keys
                else:
                    self.data[key] = self.next_data[key] # Create the new key and merge the data
            self.received = False
            self.next_data = {}

            # Continuously get the motion values from the CV handler output
            yaw = self.data["dhd_approach_cv"].get("yaw", 0)
            forward = self.data["dhd_approach_cv"].get("forward", 0)
            end = self.data["dhd_approach_cv"].get("end", False)

            if end:
                break

            self.robot_control.movement(yaw=yaw, forward=forward)
            print(f"[DEBUG] yaw: {yaw}, forward: {forward}")
        
        print("[INFO] DHD approach mission end")


    def cleanup(self):
        """
        Clean up the DHD approach mission
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name) # Stop the CV script

        # Idle the robot
        self.robot_control.movement()
        print("[INFO] DHD Approach mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package
    
    import time
    from auv.utils import deviceHelper

    rospy.init_node("dhd_approach", anonymous=True)

    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = DHDApproachMission(**config)

    # Run the mission
    mission.run()
    time.sleep(2)
    mission.cleanup()
