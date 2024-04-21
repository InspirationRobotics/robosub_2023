"""
Handles the running of the Gate mission
"""

import json
import time

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running CV scripts
from ..motion import robot_control # For moving the sub
from ..utils import disarm # For disarming the sub


class GateMission:
    """
    Class to handle the Gate mission
    """
    cv_files = ["gate_cv"]

    def __init__(self, target="abydos", **config):
        """
        Initializes the GateMission class

        Args:
            target: the side of the gate to pass through (either Earth or Abydos)
            config: 
        """
        self.config = config
        self.data = {}  # dict to store the data from the cv handlers
        self.next_data = {}  # dict to store the data from the cv handlers
        self.received = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)
        # init the cv handlers
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        # set target for gate
        self.cv_handler.set_target("gate_cv", target)
        print("[INFO] Gate mission init")

    def callback(self, msg):
        """Callback for the cv_handler output, you can have multiple callback for multiple cv_handler"""
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

        #print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """

        while not rospy.is_shutdown():
            if not self.received:
                continue
            
            self.robot_control.set_depth(0.8)
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            if not "gate_cv" in self.data.keys():
                continue

            # get the lateral and forward values from the cv (if they exist)
            lateral = self.data["gate_cv"].get("lateral", None)
            forward = self.data["gate_cv"].get("forward", None)
            yaw = self.data["gate_cv"].get("yaw", None)
            end = self.data["gate_cv"].get("end", False)
            #if any(i == None for i in (lateral, forward, yaw)):
            #    continue
            # direcly feed the cv output to the robot control
            if end:
                print("Ending...")
                self.robot_control.forwardDist(6, 2)
                self.robot_control.movement(lateral=0, yaw=0, forward=0)
                break
            else:
                self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw)
                print(lateral, forward, yaw)

        print("[INFO] Gate mission run")

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

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
    mission = GateMission(**config)

    # Run the mission
    mission.run()
    mission.cleanup()
