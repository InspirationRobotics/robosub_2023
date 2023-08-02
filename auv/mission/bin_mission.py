"""
Template file to create a mission class
"""

# import what you need from within the package

import json

import rospy

from ..device import cvHandler
from ..motion import robot_control
from ..motion.servo import Dropper


class BinMission:
    cv_files = ["bin_cv"]

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
        self.cv_handler = cvHandler.CVHandler(**self.config)

        # init the cv handlers
        # dummys are used to input a video file instead of the camera
        dummys = self.config.get("cv_dummy", [None] * len(self.cv_files))
        for file_name, dummy in zip(self.cv_files, dummys):
            self.cv_handler.start_cv(file_name, self.callback, dummy_camera=dummy)

        # init variables for the mission
        self.dropper = Dropper()
        self.ball_count = 0
        self.time_going_down = None
        self.time_dropping = None
        self.time_going_up = None

        print("[INFO] Template mission init")

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

        # here is an example of how to set a target
        self.cv_handler.set_target("bin_cv", "Earth")

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

            # get the lateral and forward values from the cv (if they exist)
            lateral = self.data["bin_cv"].get("lateral", None)
            yaw = self.data["bin_cv"].get("yaw", None)
            forward = self.data["bin_cv"].get("forward", None)
            is_aligned = self.data["bin_cv"].get("drop", None)

            # we are aligned, we are moving down to get closer to the bin
            if is_aligned and not self.time_going_down:
                self.rc.set_depth(1.5)
                self.time_going_down = time.time()

            # we are aligned, we are dropping the ball
            elif is_aligned and self.time_going_down - time.time() > 5:
                self.dropper.drop()
                self.ball_count += 1
                print(f"[BIN MISSION] Dropping ball #{self.ball_count}")
                self.time_dropping = time.time()

            # wait for us to see the ball drop
            elif self.time_dropping and time.time() - self.time_dropping > 3:
                self.rc.set_depth(0.75)
                print("[BIN MISSION] Going up")
                self.time_going_up = time.time()

            # wait for us to go up = end of mission
            elif self.time_going_up and time.time() - self.time_going_up > 3:
                print("[BIN MISSION] Mission complete")
                break

            else:
                self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw)
                print(f"[BIN MISSION] lateral: {lateral}, forward: {forward}, yaw: {yaw}")

        self.robot_control.movement()

    def cleanup(self):
        """
        Here should be all the code required after the run fonction.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # idle the robot
        self.robot_control.movement()
        rospy.signal_shutdown("End of mission")
        print("[INFO] Bin mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package
    import time
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
    mission = BinMission(**config)

    # Run the mission
    mission.run()
    time.sleep(2)
    mission.cleanup()
