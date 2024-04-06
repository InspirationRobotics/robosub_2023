"""
To control the robot by setting PWM values for each thruster. The class RobotControl publishes PWM values to the MAVROS topics.
These MAVROS topics are predefined topics that the pixhawk subscribes to. RobotControl does not handle the interface between the 
pixhawk flight controller and the software -- that is the job that pixstandalone.py does. 
"""

import time

# Import the MAVROS message types that are needed
import mavros_msgs.msg
import mavros_msgs.srv
import rospy
from std_msgs.msg import Float64, Float32MultiArray

# Import the PID controller
from simple_pid import PID

# Get the mathematical functions that handle various navigation tasks from utils.py
from .utils import get_distance, get_heading_from_coords, heading_error, rotate_vector, inv_rotate_vector
from ..utils import deviceHelper # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)
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
    def setHeadingOld(self, target: int):
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        target = (target) % 360
        # dir variable is direction; clockwise and counterclockwise
        while not rospy.is_shutdown():
            current = int(self.compass)
            print(current)
            dir = 1  # cw
            diff = abs(target - current)
            if diff >= 180:
                dir *= -1
            if current > target:
                dir *= -1
            if diff >= 180:
                diff = 360 - diff
            # if farther from desired heading, speed will be faster, if closer to heading, speed will decrease
            if diff <= 10:
                speed = 55
            else:
                speed = 70
            # once compass is within 2 degrees of desired heading, stop sending pwms to yaw
            if diff <= 1:
                pwm.channels[3] = 1500
                self.pub_thrusters.publish(pwm)  # publishing pwms to stop yawing
                break
            else:
                pwm.channels[3] = 1500 + (dir * speed)
                self.pub_thrusters.publish(pwm)  # publishing pwms to continue yawing

    def navigate_dvl(self, x, y, z, end_heading=None, relative_coord=True, relative_heading=True, update_freq=10):
        """
        # NOTE: this function is complex and requires compass to work PERFECTLY
        if you want to simply move forward, use forward_dvl instead

        Navigate to a given point, blocking function, compass should be calibrated 
        x, y are in meters, by default relative to the current position and heading ; z is absolute
        x = lateral, y = forward, z = depth

        end_heading (optionnal) is the heading to reach at the end of the navigation
        it defaults to the heading to reach the target point from start in a straight line

        update_freq is the frequency at which the PID controllers are updated
        """

        if self.dvl is None:
            print("[ERROR] DVL not available, cannot navigate")
            return

        # reset PID integrals
        for pid in self.PIDs.values():
            pid.reset()

        if not relative_coord:
            # convert to relative coordinates
            # heading 0 is north so y "+" axis is going to the north
            x -= self.dvl.position[0]
            y -= self.dvl.position[1]

        if relative_heading:
            # rotate [x, y] according to the current heading
            x, y = rotate_vector(x, y, self.compass)

        target_heading = get_heading_from_coords(x, y)
        print(f"[INFO] Navigating to {x}, {y}, {z}, {target_heading}deg current {self.compass}")

        # rotate and set depth
        self.set_heading(target_heading)
        self.set_depth(z)

        # enter a local scope to handle coordinates nicely
        with self.dvl:
            # set the depth independently
            self.set_depth(z)

            # navigate to the target point
            while not rospy.is_shutdown():
                if not self.dvl.is_valid:
                    print("[WARN] DVL data not valid, skipping")
                    time.sleep(0.5)
                    continue

                # ensure position data is updated
                if not self.dvl.data_available:
                    continue
                self.dvl.data_available = False

                err_x, err_y = inv_rotate_vector(
                    x - self.dvl.position[0],
                    y - self.dvl.position[1],
                    self.compass,
                )

                # check if we reached the target
                x_err_th = 0.1 + self.dvl.error[0]
                y_err_th = 0.1 + self.dvl.error[1]
                if abs(err_x) <= x_err_th and abs(err_y) <= y_err_th:
                    print("[INFO] Target reached")
                    break

                # calculate PID outputs
                output_x = self.PIDs["lateral"](-err_x)
                output_y = self.PIDs["forward"](-err_y)
                print(f"[DEBUG] err_x={err_x}, err_y={err_y}, output_x={output_x}, output_y={output_y}")
                self.movement(lateral=output_x, forward=output_y)

    def forward_dvl(self, throttle, distance):
        """Move forward using the DVL, blocking function"""
        if self.dvl is None:
            print("[ERROR] DVL not available, cannot navigate")
            return

        print(f"[INFO] Moving forward {distance}m at throttle {throttle}")

        # enter a local scope to handle coordinates nicely
        with self.dvl:
            # navigate to the target point
            while not rospy.is_shutdown():
                if not self.dvl.is_valid:
                    print("[WARN] DVL data not valid, skipping")
                    time.sleep(0.5)
                    continue

                # ensure position data is updated
                if not self.dvl.data_available:
                    continue
                self.dvl.data_available = False

                y = self.dvl.position[1]
                error = distance - y

                # check if we reached the target
                if abs(error) <= 0.1:
                    print("[INFO] Target reached")
                    break
                
                forward_output = np.clip(error * 4, -throttle, throttle)
                print(f"[DEBUG] error={error}, forward_output={forward_output}")

                self.movement(forward=forward_output)

    def lateral_dvl(self, throttle, distance):
        """Move laterally using the DVL, blocking function"""
        if self.dvl is None:
            print("[ERROR] DVL not available, cannot navigate")
            return

        print(f"[INFO] Moving laterally {distance}m at throttle {throttle}")

        # enter a local scope to handle coordinates nicely
        with self.dvl:
            # navigate to the target point
            while not rospy.is_shutdown():
                if not self.dvl.is_valid:
                    print("[WARN] DVL data not valid, skipping")
                    time.sleep(0.5)
                    continue

                # ensure position data is updated
                if not self.dvl.data_available:
                    continue
                self.dvl.data_available = False

                x = self.dvl.position[0]
                error = distance - x

                # check if we reached the target
                if abs(error) <= 0.1:
                    print("[INFO] Target reached")
                    break
                
                lateral_output = np.clip(error * 4, -throttle, throttle)
                print(f"[DEBUG] error={error}, lateral_output={lateral_output}")

                self.movement(lateral=lateral_output)

    def forwardHeading(self, power, t):
        # Power 1: 7.8t+3.4 (in inches)
        # Power 2: 21t+0.00952
        # Power 3: 32.1t-18.7
        forwardPower = (power * 80) + 1500  # calculating power for forward thrusters
        if t > 3:  # designating time for backstopping function to prevent inertia from increasing distance travelled
            timeStop = t / 6
        else:
            timeStop = 0.5

        # t = t+timeStop #this doesn't make sense because this increases time over long distance
        powerStop = 1500 - (power * 40)
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[4] = forwardPower
        startTime = time.time()
        while time.time() - startTime < t:
            self.pub_thrusters.publish(pwm)  # publishing pwms for forward commands to thrusters
            time.sleep(0.1)

        print("[INFO] finished forward")
        # calculating gradual decrease for thruster power
        gradDec = int((forwardPower - powerStop) / (timeStop * 10))
        startTime = time.time()
        # gradually decreasing forward thruster power for designated time calculated above
        while time.time() - startTime < timeStop:
            current_p = pwm.channels[4]
            pwm.channels[4] = current_p - gradDec
            self.pub_thrusters.publish(pwm)  # publishing reduced pwms for forward thrust
            time.sleep(0.1)

        t2 = 0
        print("[INFO] finished backstopping")
        pwm.channels = [1500] * 18
        startTime = time.time()
        # publishing idle pwms to sub to stop moving
        while time.time() - startTime <= 0.5:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.05)
        print("[INFO] finished forward heading")

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

    # incorporates universal backstop for forward movement
    def forwardUni(self, power, t):
        forwardPower = (power * 80) + 1500
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[4] = int(forwardPower)
        if config.get("sub", "onyx") == "graey":
            pwm.channels[5] = 1500-int(power*7)
        startTime = time.time()
        while time.time() - startTime < t:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.1)
        self.backStop(pwm, t)

    def mapping(self, x, in_min, in_max, out_min, out_max):
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
        self.forwardUni(power, time)
