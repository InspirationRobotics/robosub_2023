"""
Handles the running of the Gate mission
Picks a side, either Abydos or Earth, to move through
"""

# import what you need from within the package

import json
import time

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For controlling the motion of the robot
from ..utils import disarm


class Gate2Mission:
    cv_files = ["gate2_cv"] # CV script to run

    def __init__(self, target="abydos", **config):
        """
        Initializes the Gate2Mission class

        Args:
            target (str): the side of the gate to pass through (either Earth or Abydos), defaults to abydos
            config: Mission-specific parameters to run the mission
        """
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handler(s)
        self.next_data = {}  # Dictionary to store the most updated data from the CV handler(s). This will later be merged with self.data.
        self.received = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Initialize the CV handler
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        # Set the target for the gate (either Earth or Abydos)
        self.cv_handler.set_target("gate2_cv", target)
        print("[INFO] Gate mission init")

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

        #print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Run the gate mission
        """

        while not rospy.is_shutdown():
            if not self.received:
                continue
            
            self.robot_control.set_depth(0.5) # Set the depth to 0.5 m

            # Merge the data from self.next_data with self.data, and wipe self.next_data so it can take the new, more updated data from
            # the CV handler.
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key]) # If the keys already match, merge the corresponding data
                else:
                    self.data[key] = self.next_data[key] # Else create the new key and merge the corresponding data
            self.received = False
            self.next_data = {}

            if not "gate2_cv" in self.data.keys():
                continue

            # Get the lateral and forward values from the CV script (if they exist)
            lateral = self.data["gate2_cv"].get("lateral", None)
            forward = self.data["gate2_cv"].get("forward", None)
            yaw = self.data["gate2_cv"].get("yaw", None)
            end = self.data["gate2_cv"].get("end", False)

            #if any(i == None for i in (lateral, forward, yaw)):
            #    continue
            # direcly feed the cv output to the robot control

            # After the mission is over, move forward a little further to ensure we actually are past the gate
            if end:
                print("Ending...")
                self.robot_control.forwardDist(8, 2)
                self.robot_control.movement(lateral=0, yaw=0, forward=0)
                break
            else:
                self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw)
                print(lateral, forward, yaw)

        print("[INFO] Gate mission run")

    def cleanup(self):
        """
        Cleans up the gate mission. Stops CV scripts and idles the sub.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name) # Stop the CV script

        # Idle the robot
        self.robot_control.movement(lateral=0, forward=0, yaw=0)
        print("[INFO] Gate mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package
    
    import time
    from auv.utils import deviceHelper

    rospy.init_node("gate_mission", anonymous=True)
    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = Gate2Mission(**config)

    # Run the mission
    mission.run()
    mission.cleanup()
