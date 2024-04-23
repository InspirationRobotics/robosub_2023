"""
Class for running the torpedo mission.
Maneuvers to be aligned with the target(s) and fires the torpedoes.
"""

import json

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For controlling the motion of the sub
from ..motion.servo import Servo # For controlling the servos (in this case the torpedo launcher)


class TorpedoMission:
    """
    Class to run the torpedo mission
    """
    cv_files = ["torpedo_cv"] # CV script to run

    def __init__(self, **config):
        """
        Initialize the TorpedoMission class

        Args:
            config: Mission-specific keyword arguments to run the mission
        """
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handler(s)
        self.next_data = {}  # Dictionary to store the most updated data from the CV handler(s). This will later be merged with self.data.
        self.received = False
        self.torpedo_1_fired = False 
        self.torpedo_2_fired = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Initialize the CV handler
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.servo = Servo()

        print("[INFO] Torpedo mission init")

    def callback(self, msg):
        """
        Callback to get cv_handler output. Converts the output to JSON format and puts it in self.next_data, the list that holds 
        the most updated CV output data.

        Args:
            msg: Data from the CV handler. These are the motion commands, servo commands, and possibly the visualized frame.
        """
        file_name = msg._connection_header["topic"].split("/")[-1] # Get the file name from the ROS topic name
        data = json.loads(msg.data) # Convert the data to JSON format
        self.data[file_name] = data
        self.next_data[file_name] = data
        self.received = True

        #print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Run the torpedo mission.
        """
        while not rospy.is_shutdown():
            try:
                if not self.received:
                    continue

                # Merge the data from self.next_data with self.data, and wipe self.next_data so it can take the new, more updated data from
                # the CV handler.
                for key in self.next_data.keys():
                    if key in self.data.keys():
                        self.data[key].update(self.next_data[key])
                    else:
                        self.data[key] = self.next_data[key]
                self.received = False
                self.next_data = {}

                if not "torpedo_cv" in self.data.keys():
                    continue

                # Get the motion commands from the CV handler
                lateral = self.data["torpedo_cv"].get("lateral", 0)
                forward = self.data["torpedo_cv"].get("forward", 0)
                vertical = self.data["torpedo_cv"].get("vertical", 0)
                yaw = self.data["torpedo_cv"].get("vertical", 0)

                # Get the commands of whether to fire the torpedo or not (default to False)
                torpedo1 = self.data["torpedo_cv"].get("fire1", False)
                torpedo2 = self.data["torpedo_cv"].get("fire2", False)

                # Fire torpedo 1 if the flag is True
                if torpedo1 and not self.torpedo_1_fired:
                    self.servo.torpedo()
                    self.torpedo_1_fired = True

                # Fire torpedo 2 if the flag is True
                if torpedo2 and not self.torpedo_2_fired:
                    self.servo.torpedo()
                    self.torpedo_2_fired = True

                # If the mission has ended, idle the sub
                if self.data["torpedo_cv"].get("end", False):
                    self.robot_control.movement()
                    break

                if any(i == None for i in (lateral, forward)):
                    continue

                # Direcly feed the CV output to the robot control class
                self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw)
                self.robot_control.set_relative_depth(vertical)
                #self.robot_control.set_depth(self.robot_control.depth + vertical)

                # here is an example of how to set a target
                # self.cv_handler.set_target("torpedo_cv", "albedo")

            except Exception as e:
                print(f"[ERROR] {e}")
                print(e)
                # Idle the robot (just in case something went wrong)
                self.robot_control.movement()
                break

    def cleanup(self):
        """
        Clean up the torpedoes mission.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        print("[INFO] Torpedo mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.torpedo_mission"
    # You can also import it in a mission file outside of the package

    import time
    from auv.utils import deviceHelper # Configuration fo the devices attached to the sub

    rospy.init_node("torpedo_mission", anonymous=True)

    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = TorpedoMission(**config)

    # Run the mission
    mission.run()

    # Clean up
    time.sleep(2)
    mission.cleanup()
