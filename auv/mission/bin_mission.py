"""
Class to be called in order to complete the Bins mission.
Drops markers into the correct bin.
"""

import json

import rospy

from ..device import cv_handler # To run the CV script.
from ..motion import robot_control # To control the motion of the sub.
from ..motion.servo import Dropper # For controlling the marker dropper.


class BinMission:
    """
    Class to handle the Bins mission.
    """
    cv_files = ["bin_cv"]

    def __init__(self, **config):
        """
        Initialize the BinMission class. 

        Args:
            config (dict): Configuration settings to run the mission
        """
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handlers.
        self.next_data = {}  # Dictionary to store the new data from the CV handlers. This data will be merged with the self.data. 
        self.received = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Dummys are used to input a video file instead of a camera stream for the CV script to run on.
        # Initialize the CV script -- start_cv()
        dummys = self.config.get("cv_dummy", [None] * len(self.cv_files))
        for file_name, dummy in zip(self.cv_files, dummys):
            self.cv_handler.start_cv(file_name, self.callback, dummy_camera=dummy)

        # Initialize various variables for the mission.
        self.dropper = Dropper()
        self.ball_count = 0
        self.time_going_down = None
        self.time_dropping = None
        self.time_going_up = None

        print("[INFO] Template mission init") # From the copied template code

    def callback(self, msg):
        """
        Method to callback the cv_handler output. Multiple callbacks are possible for multiple cv_handlers. Converts the message data to a JSON format.
        
        Args:
            msg: Message data from the particular ROS topic. This is the dictionary of motion values that tells the sub how to move, and/or the visualized frame
        """
        file_name = msg._connection_header["topic"].split("/")[-1] # Get the file name from the topic name
        data = json.loads(msg.data) # Get the message data.
        self.next_data[file_name] = data # Puts the new CV handler output in self.next_data
        self.received = True

    def run(self):
        """
        Run the Bins mission. Does everything in a sequential and defined way. 
        """

        # Set the target of where to drop the markers. In this case, the target is the "Earth" bins.
        self.cv_handler.set_target("bin_cv", "Earth")

        while not rospy.is_shutdown():
            if not self.received:
                continue

            # Put the CV_handler data in self.data (with the correct key), and wipe next_data to receive the new inputs from the CV Handler.
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            # Get the motion values for the sub, if they exist.
            lateral = self.data["bin_cv"].get("lateral", None)
            yaw = self.data["bin_cv"].get("yaw", None)
            forward = self.data["bin_cv"].get("forward", None)
            is_aligned = self.data["bin_cv"].get("drop", None)

            # We are aligned and are moving down to get closer to the bin.
            if is_aligned and not self.time_going_down:
                self.robot_control.set_depth(1.5)
                self.time_going_down = time.time()

            # Wait 5 seconds after we have gone down to the bin, then drop the ball.
            elif is_aligned and self.time_going_down - time.time() > 5:
                self.dropper.drop()
                self.ball_count += 1
                print(f"[BIN MISSION] Dropping ball #{self.ball_count}")
                self.time_dropping = time.time()

            # Give three seconds to wait after the ball has dropped, then rise.
            elif self.time_dropping and time.time() - self.time_dropping > 3:
                self.robot_control.set_depth(0.75)
                print("[BIN MISSION] Going up")
                self.time_going_up = time.time()

            # Say that the mission has been completed 3 seconds after we have risen.
            elif self.time_going_up and time.time() - self.time_going_up > 3:
                print("[BIN MISSION] Mission complete")
                break

            else:
                self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw)
                print(f"[BIN MISSION] lateral: {lateral}, forward: {forward}, yaw: {yaw}")

        self.robot_control.movement()

    def cleanup(self):
        """
        Clean up the mission. Stop the CV script and idle the robot.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name) # Stop the CV script

        # Idle the robot
        self.robot_control.movement()
        rospy.signal_shutdown("End of mission")
        print("[INFO] Bin mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package
    from auv.utils import deviceHelper

    rospy.init_node("bin_mission", anonymous=True)
    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = BinMission(**config) # Unpacks config into keyword arguments

    # Run the mission
    mission.run()
    time.sleep(2)
    mission.cleanup()
