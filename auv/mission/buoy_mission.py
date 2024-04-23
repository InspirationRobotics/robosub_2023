"""
Class for running the Buoy misson.
Bumps the correct characters on the buoy.
"""

import json

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For controlling the thrusters of the sub
import time


class BuoyMission:
    """
    Run the Buoy mission
    """
    cv_files = ["buoy_cv"] # CV script to run

    def __init__(self, target="earth1", **config):
        """
        Initialize the BuoyMission class

        Args:
            config (dict): Configuration settings to run the mission
        """
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handler(s)
        self.next_data = {}  # Dictionary to store the most updated data from the CV handler(s)
        self.received = False
        self.target = target
        self.side = None
        self.baseDepth = 3 # 2.3 (bottom of buoy)
        self.upDepth = 2.4 # 1.8 (top of buoy)
            
        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # Initialize the CV script
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)
        self.cv_handler.set_target("buoy_cv", self.target)
        print("[INFO] Buoy mission init")

    def callback(self, msg):
        """
        Callback function for obtaining output from the CV handler. Multiple callback functions can be used for multiple CV handlers. Converts the 
        message data to JSON format.

        Args:
            msg: Output from the CV handler. This will be a dictionary of motion values, possible the visualized frame, and potentially servo commands
        """
        file_name = msg._connection_header["topic"].split("/")[-1] # Get the file name from the topic name
        data = json.loads(msg.data) # Convert to JSON format
        self.next_data[file_name] = data
        self.received = True

        # print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Run the buoy mission
        """

        self.robot_control.set_depth(self.baseDepth) # Move to the bottom of the buoy
        time.sleep(4)

        while not rospy.is_shutdown():
            if not self.received:
                continue

            # Merge self.next_data, which contains the updated CV handler output, with self.data, which contains the previous CV handler output.
            # self.next_data will be wiped so that it can be updated with the new CV handler output.
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            self.cv_handler.set_target("buoy_cv", self.target) # Set the target of the CV script (not really a physical "target" in this case)
            if not "buoy_cv" in self.data.keys(): 
                continue

            if self.data["buoy_cv"].get("end", False):
                # TODO: go up and forward
                print("Ended")
                # self.robot_control.set_depth(1)
                # time.sleep(2)
                # self.robot_control.set_depth(0.5)
                # time.sleep(2)
                # self.robot_control.forwardDist(1, 1.2)
                break
            
            # If the buoy mission is running, and the target is an actual target, run a preset motion routine to bump the correct characters
            if self.data["buoy_cv"].get("finished", False) and self.target != "board":
                print("Doing bump procedure")
                # TODO: go forward, bump, back, up, opposite diagonal, bump, back
                self.robot_control.forwardDist(3.2, 2) #1.6
                print("Finished forward, going back")
                self.robot_control.forwardDist(3.2, -2)
                print("Bump 1 complete")
                if self.side != None:
                    self.robot_control.set_depth(self.upDepth)
                    time.sleep(4)
                    if self.side == 0: # 0 is left and 1 is right (for og target)
                        print("Going to right side")
                        self.robot_control.lateralUni(1.5,5)
                    else:
                        print("Going to left side")
                        self.robot_control.lateralUni(-1.5,5)
                    print("Now going forward")
                    self.robot_control.forwardDist(4.4, 2) # bump it 4.4
                    print("Bumped other side")
                    self.robot_control.forwardDist(4, -2) # go back
                print("Done bumping, aligning with board")
                self.target = "board" # Realign with the board

            # Take the output from the CV handler -- second method
            self.cv_handler.set_target("buoy_cv", self.target)
            lateral = self.data["buoy_cv"].get("lateral", None)
            yaw = self.data["buoy_cv"].get("yaw", None)
            forward = self.data["buoy_cv"].get("forward", None)
            vertical = self.data["buoy_cv"].get("vertical", None)
            if self.data["buoy_cv"].get("targetSide", None) != None:
                self.side = self.data["buoy_cv"].get("targetSide")
                #print(f"Got side:{self.side}")

            # If forward or lateral == None, then simply continue
            if any(i == None for i in (lateral, forward)):
                continue
            self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw, vertical=0)
            #print(forward, lateral, yaw, vertical)

        print("[INFO] Buoy mission run")

    def cleanup(self):
        """
        Clean up after the surfacing mission. Stops the CV script, and idles the robot.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name) # Stop the CV script

        # Idle the robot
        self.robot_control.movement()
        print("[INFO] Buoy mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.buoy_mission"
    # You can also import it in a mission file outside of the package
    import time
    from auv.utils import deviceHelper
    # from ..motion import robot_control
    # import rospy

    # rospy.init_node("buoy_mission", anonymous=True)
    # rc = robot_control.RobotControl()
    # time.sleep(2)
    # rc.forwardDist(1.5, -2) #dist power
    # # time.sleep(1)
    # # rc.lateralUni(1.2,4) #power time
    # print("Finished forward")

    rospy.init_node("buoy_mission", anonymous=True)

    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = BuoyMission(**config)

    # Run the mission
    mission.run()
    time.sleep(2)
    mission.cleanup()
