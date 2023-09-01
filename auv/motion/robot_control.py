
import time

import mavros_msgs.msg
import mavros_msgs.srv
import rospy
from std_msgs.msg import Float64, Float32MultiArray
from simple_pid import PID

from .utils import get_distance, get_heading_from_coords, heading_error, rotate_vector, inv_rotate_vector
from ..utils import deviceHelper
from ..device.dvl import dvl
import math
import numpy as np

config = deviceHelper.variables

class RobotControl:
    """Class to control the robot"""

    def __init__(self, enable_dvl=True):
        # init some variables
        self.config = config
        self.depth = self.config.get("INIT_DEPTH", 0.0)
        self.compass = None

        # # dvl sensor setup (both subs)
        # if enable_dvl:
        #     self.dvl = dvl.DVL()
        #     self.dvl.start()
        # else:
        self.dvl = None

        # establishing thrusters and depth publishers
        self.sub_compass = rospy.Subscriber("/auv/devices/compass", Float64, self.get_callback_compass())
        self.sub_depth = rospy.Subscriber("/auv/devices/baro", Float32MultiArray, self.callback_depth)
        self.pub_thrusters = rospy.Publisher("auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn, queue_size=10)
        self.pub_depth = rospy.Publisher("auv/devices/setDepth", Float64, queue_size=10)
        self.pub_rel_depth = rospy.Publisher("auv/devices/setRelativeDepth", Float64, queue_size=10)
        # TODO: reset pix standalone depth Integration param 

        # set of PIDs to handle movement of the robot
        self.PIDs = {
            "yaw": PID(
                self.config.get("YAW_PID_P", 10),
                self.config.get("YAW_PID_I", 0.01),
                self.config.get("YAW_PID_D", 0.0),
                setpoint=0,
                output_limits=(-2, 2),
            ),
            "forward": PID(
                self.config.get("FORWARD_PID_P", 4.0),
                self.config.get("FORWARD_PID_I", 0.01),
                self.config.get("FORWARD_PID_D", 0.1),
                setpoint=0,
                output_limits=(-2, 2),
            ),
            "lateral": PID(
                self.config.get("LATERAL_PID_P", 4.0),
                self.config.get("LATERAL_PID_I", 0.01),
                self.config.get("LATERAL_PID_D", 0.0),
                setpoint=0,
                output_limits=(-2, 2),
            ),
        }

        # wait for the topics to be up
        time.sleep(1)

    def get_callback_compass(self):
        def _callback_compass(msg):
            """Get compass heading from /auv/devices/compass topic"""
            self.compass = msg.data

        def _callback_compass_dvl(msg):
            """Get compass heading from dvl"""
            self.compass = msg.data
            self.dvl.compass_rad = math.radians(msg.data)

        if self.dvl:
            return _callback_compass_dvl
        else:
            return _callback_compass

    def callback_depth(self, msg):
        """Get depth data from barometer /auv/devices/baro topic"""
        self.depth = msg.data[0]

    def set_depth(self, d):
        """Set depth to a given value"""
        depth = Float64()
        depth.data = d
        self.pub_depth.publish(depth)
        print(f"[INFO] Depth set to {d}")

    def set_relative_depth(self, delta_depth):
        """Set depth to a relative value"""
        rel_depth = Float64()
        rel_depth.data = delta_depth
        self.pub_rel_depth.publish(rel_depth)
        print(f"[INFO] Changing Depth relatively by {delta_depth}, current {self.depth}")

    def movement(
        self,
        yaw=None,
        forward=None,
        lateral=None,
        pitch=None,
        roll=None,
        vertical=0,
        **kwargs,
    ):
        """
        Move the robot in a given direction, non blocking function
        Inputs are from -5 to 5
        This controls directly the pwm of the thrusters, no feedback from dvl involved

        # TODO Handle timeout of the pixhawk
        """

        pwm = mavros_msgs.msg.OverrideRCIn()

        channels = [1500] * 18
        # channels[2] = int((vertical * 80) + 1500) if vertical else 1500
        channels[3] = int((yaw * 80) + 1500) if yaw else 1500
        channels[4] = int((forward * 80) + 1500) if forward else 1500
        channels[5] = int((lateral * 80) + 1500) if lateral else 1500
        channels[6] = int((pitch * 80) + 1500) if pitch else 1500
        channels[7] = int((roll * 80) + 1500) if roll else 1500
        pwm.channels = channels

        # publishing pwms to /auv/devices/thrusters
        if vertical!=0: self.set_relative_depth(vertical)
        self.pub_thrusters.publish(pwm)
    def set_heading(self, target: int):
        """Yaw to target heading, heading is absolute, blocking function"""

        target = (target) % 360
        print(f"[INFO] Setting heading to {target}")

        while not rospy.is_shutdown():
            if self.compass is None:
                print("[WARN] Compass not ready")
                time.sleep(0.5)
                continue

            error = heading_error(self.compass, target)
            # normalize error to -1, 1 for the PID controller
            output = self.PIDs["yaw"](-error / 180)

            print(f"[DEBUG] Heading error: {error}, output: {output} {self.compass} {target}")

            if abs(error) <= 1:
                print("[INFO] Heading reached")
                break

            self.movement(yaw=output)
            time.sleep(0.1)

        print(f"[INFO] Finished setting heading to {target}")

    # set heading is a function that yaws until reaches desired heading input integer
    
    # incorporates universal backstop function for lateral movement
    def lateralUni(self, power, t):
        forwardPower = (power * 80) + 1500
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[5] = int(forwardPower)
        startTime = time.time()
        while time.time() - startTime < t:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.1)
        self.backStop(pwm, t)
        
    def waitTimer(t):
        startTime = time.time()
        while time.time() - startTime < t:
            return False
        return True
    
    # incorporates universal backstop for forward movement
    def forwardUni(self, power, t):
        if config.get("sub", "onyx") == "graey":
            lateral = -power*7/80
        while not self.waitTimer(t):
           self.movement(forward = power, lateral = lateral)
           time.sleep(0.1)
        self.movement()
       # self.backStop(pwm, t)
    '''def mapping(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    # universal backstop that is given pwm to reduce and overall time of movement which calculates inertia and sends commands/pwms for thrust in the negative direction for backstopping
    def backStop(self, pwm, t):
        if t > 3:
            timeStop = t / 6
        else:
            timeStop = 0.5
        maxPower = [1500] * 18
        powerStop = [1500] * 18
        for i in range(len(pwm.channels)):
            if pwm.channels[i] != 1500:
                maxPower[i] = pwm.channels[i]
                powerStop[i] = (1500 - pwm.channels[i]) / 2 + 1500
        startTime = time.time()
        while time.time() - startTime <= timeStop:
            for i in range(len(pwm.channels)):
                if pwm.channels[i] != 1500:
                    pwm.channels[i] = int(
                        self.mapping(
                            time.time() - startTime,
                            0,
                            timeStop,
                            maxPower[i],
                            powerStop[i],
                        )
                    )
            self.pub_thrusters.publish(pwm)
            time.sleep(0.05)
        pwm.channels = [1500] * 18
        startTime = time.time()
        while time.time() - startTime <= 0.5:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.05)
        print("[INFO] finished backstopping")

    # uses functions from testing to correlate pwms and time to distance and then utilizes forwardHeadingUni commmand to send pwms to thrusters for calculated pwms and time
    #personally i do not see a point in this, just spam time
    def forwardDist(self, dist, power):
        inches = 39.37 * dist
        print(power, inches)
        eqPower = abs(round(power))
        time = 0
        if eqPower >= 3:
            inches = inches - 9.843
            time = (inches + 18.7) / 32.1
        elif eqPower == 2:
            time = (inches - 0.01) / 21
        elif eqPower == 1:
            time = (inches - 3.4) / 7.8
        self.forwardUni(power, time)'''
