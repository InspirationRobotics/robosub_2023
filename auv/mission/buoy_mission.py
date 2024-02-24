"""
Template file to create a mission class
"""

# import what you need from within the package

import json

import rospy
from std_msgs.msg import String

from ..device import cv_handler
from ..motion import robot_control
import time


class BuoyMission:
    cv_files = ["buoy_cv"]

    def __init__(self, target="earth1", **config):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        config is a dict containing the settings you give to the mission
        """
        self.config = config
        self.data = {}  # dict to store the data from the cv handlers
        self.next_data = {}  # dict to store the data from the cv handlers
        self.received = False
        self.target = target
        self.side = None
        self.baseDepth = 3# 2.3 (bottom of buoy)
        self.upDepth = 2.4 #1.8 (top of buoy)
            
        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # init the cv handlers
        # dummys are used to input a video file instead of the camera
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)
        self.cv_handler.set_target("buoy_cv", self.target)
        print("[INFO] Buoy mission init")

    def callback(self, msg):
        """Callback for the cv_handler output, you can have multiple callback for multiple cv_handler"""
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

        # print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """

        self.robot_control.set_depth(self.baseDepth)
        time.sleep(4)

        while not rospy.is_shutdown():
            if not self.received:
                continue
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key])
                else:
                    self.data[key] = self.next_data[key]
            self.received = False
            self.next_data = {}

            self.cv_handler.set_target("buoy_cv", self.target)
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
            
            if self.data["buoy_cv"].get("finished", False) and self.target != "board":
                print("Doing bump procedure")
                # TODO: go forward, bump, back, up, opposite diagnol, bump, back
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
                self.target = "board" # realign to board

            self.cv_handler.set_target("buoy_cv", self.target)
            lateral = self.data["buoy_cv"].get("lateral", None)
            yaw = self.data["buoy_cv"].get("yaw", None)
            forward = self.data["buoy_cv"].get("forward", None)
            vertical = self.data["buoy_cv"].get("vertical", None)
            if self.data["buoy_cv"].get("targetSide", None) != None:
                self.side = self.data["buoy_cv"].get("targetSide")
                #print(f"Got side:{self.side}")

            if any(i == None for i in (lateral, forward)):
                continue
            self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw, vertical=0)
            #print(forward, lateral, yaw, vertical)

        print("[INFO] Buoy mission run")

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # idle the robot
        self.robot_control.movement()
        print("[INFO] Buoy mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
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
