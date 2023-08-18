"""
Template file to create a mission class
"""

# import what you need from within the package

import json

import rospy
from std_msgs.msg import String

from ..device import cv_handler
from ..motion import robot_control
from ..motion.servo import Servo


class TorpedoMission:
    cv_files = ["torpedo_cv"]

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
        self.torpedo_1_fired = False
        self.torpedo_2_fired = False

        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cv_handler.CVHandler(**self.config)

        # init the cv handlers
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.servo = Servo()

        print("[INFO] Torpedo mission init")

    def callback(self, msg):
        """Callback for the cv_handler output, you can have multiple callback for multiple cv_handler"""
        file_name = msg._connection_header["topic"].split("/")[-1]
        data = json.loads(msg.data)
        self.data[file_name] = data
        self.next_data[file_name] = data
        self.received = True

        #print(f"[DEBUG] Received data from {file_name}")

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """
        while not rospy.is_shutdown():
            try:
                if not self.received:
                    continue

                for key in self.next_data.keys():
                    if key in self.data.keys():
                        self.data[key].update(self.next_data[key])
                    else:
                        self.data[key] = self.next_data[key]
                self.received = False
                self.next_data = {}

                if not "torpedo_cv" in self.data.keys():
                    continue
                lateral = self.data["torpedo_cv"].get("lateral", 0)
                forward = self.data["torpedo_cv"].get("forward", 0)
                vertical = self.data["torpedo_cv"].get("vertical", 0)
                yaw = self.data["torpedo_cv"].get("vertical", 0)

                torpedo1 = self.data["torpedo_cv"].get("fire1", False)
                torpedo2 = self.data["torpedo_cv"].get("fire2", False)

                if torpedo1 and not self.torpedo_1_fired:
                    # Fire torpedo 1
                    self.servo.torpedo()
                    self.torpedo_1_fired = True

                if torpedo2 and not self.torpedo_2_fired:
                    # Fire torpedo 2
                    self.servo.torpedo()
                    self.torpedo_2_fired = True

                if self.data["torpedo_cv"].get("end", False):
                    # idle the robot
                    self.robot_control.movement()
                    break


                if any(i == None for i in (lateral, forward)):
                    continue
                # direcly feed the cv output to the robot control
                self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw)
                self.robot_control.set_relative_depth(vertical)
                #self.robot_control.set_depth(self.robot_control.depth + vertical)

                # here is an example of how to set a target
                # self.cv_handler.set_target("torpedo_cv", "albedo")

            except Exception as e:
                print(f"[ERROR] {e}")
                print(e)
                # idle the robot (just in case something went wrong)
                self.robot_control.movement()
                break

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
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
    from auv.utils import deviceHelper

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

    # Cleanup
    time.sleep(2)
    mission.cleanup()
