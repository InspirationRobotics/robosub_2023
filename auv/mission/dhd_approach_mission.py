"""
Template file to create a mission class
"""

# import what you need from within the package

import json

import rospy
from std_msgs.msg import String

from ..device import cv_handler
from ..motion import robot_control


class DHDApproachMission:
    cv_files = ["dhd_approach_cv"]

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

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # init the cv handlers
        # dummys are used to input a video file instead of the camera
        dummys = self.config.get("cv_dummy", [None] * len(self.cv_files))
        for file_name, dummy in zip(self.cv_files, dummys):
            self.cv_handler.start_cv(file_name, self.callback, dummy_camera=dummy)

        print("[INFO] dvl approach mission init")

    def callback(self, msg):
        """Callback for the cv_handler output, you can have multiple callback for multiple cv_handler"""
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.next_data[file_name] = data
        self.received = True

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """

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
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # idle the robot
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
