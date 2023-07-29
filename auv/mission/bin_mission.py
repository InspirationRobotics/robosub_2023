"""
Template file to create a mission class
"""

# import what you need from within the package

import json

import rospy

from ..device import cvHandler
from ..motion import robot_control
from ..motion.servo import Servo


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

        rospy.init_node("bin_mission", anonymous=True)
        self.robot_control = robot_control.RobotControl()
        self.cv_handler = cvHandler.CVHandler()

        # init the cv handlers
        # dummys are used to input a video file instead of the camera
        dummys = self.config.get("cv_dummy", [None] * len(self.cv_files))
        for file_name, dummy in zip(self.cv_files, dummys):
            self.cv_handler.start_cv(file_name, self.callback, dummy_camera=dummy)

        # init variables for the mission
        self.servo = Servo()
        self.ball_count = 0

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
        self.cv_handler.set_target("bin_cv", "Bin")

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

            if self.data["bin_cv"].get("end", None):
                # idle the robot
                print("ending True")
                self.robot_control.movement()
                break

            # get the lateral and forward values from the cv (if they exist)
            lateral = self.data["bin_cv"].get("lateral", None)
            yaw = self.data["bin_cv"].get("yaw", None)
            forward = self.data["bin_cv"].get("forward", None)
            vertical = self.data["bin_cv"].get("vertical", None)
            do_drop = self.data["bin_cv"].get("drop", None)


            if do_drop:
                if self.ball_count == 1:
                    print(f"[BIN MISSION] already dropped 1 ball, 2nd drop not implemented yet")
                    # print(f"[BIN MISSION] ending mission")
                    # break
                    pass

                self.servo.dropper()
                self.ball_count += 1
                print(f"[BIN MISSION] Dropping ball #{self.ball_count}")
                continue

            if None in (lateral, forward):
                continue

            self.robot_control.movement(lateral=lateral, forward=forward, yaw=yaw, vertical=vertical)
            print(forward, lateral, yaw, vertical)


        print("[INFO] Bin mission end")

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
        print("[INFO] Template mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package
    import time

    config = {
        # # this dummy video file will be used instead of the camera if uncommented
        # "cv_dummy": ["/somepath/thisisavideo.mp4"],
    }

    # Create a mission object with arguments
    mission = BinMission(**config)

    # Run the mission
    mission.run()
    time.sleep(2)
    mission.cleanup()
