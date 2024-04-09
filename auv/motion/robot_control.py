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
from ..device.dvl import dvl # DVL class that enables position estimation
import math
import numpy as np

config = deviceHelper.variables # Get the configuration of the devices plugged into the sub(thrusters, camera, etc.)

class RobotControl:
    """
    Class to control the robot
    """

    def __init__(self, enable_dvl=True):
        """
        Initialize the RobotControl class

        Args:
            enable_dvl (bool): Flag to enable or disable DVL
        """

        # Initialize the configuration of the devices, depth of the sub, compass of the sub, DVL
        self.config = config
        self.depth = self.config.get("INIT_DEPTH", 0.0)
        self.compass = None

        # # dvl sensor setup (both subs)
        # if enable_dvl:
        #     self.dvl = dvl.DVL()
        #     self.dvl.start()
        # else:
        self.dvl = None

        # Establish thruster and depth publishers
        self.sub_compass = rospy.Subscriber("/auv/devices/compass", Float64, self.get_callback_compass())
        self.sub_depth = rospy.Subscriber("/auv/devices/baro", Float32MultiArray, self.callback_depth)
        self.pub_thrusters = rospy.Publisher("auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn, queue_size=10)
        self.pub_depth = rospy.Publisher("auv/devices/setDepth", Float64, queue_size=10)
        self.pub_rel_depth = rospy.Publisher("auv/devices/setRelativeDepth", Float64, queue_size=10)
        
        # TODO: reset pix standalone depth Integration param 

        # A set of PIDs (Proportional - Integral - Derivative) to handle the movement of the sub
        """
        PIDs work by continously computing the error between the desired setpoint (desired yaw angle, forward velocity, etc.) and the 
        actual value. Based on this error, PIDs generate control signals to adjust the robot's actuators (in this case thrusters) to 
        minimize the desired setpoint. 

        Video: https://www.youtube.com/watch?v=wkfEZmsQqiA

        These definitions "tune" the PID controller for the necessities of the sub -- Proportional is tuned up high, which means greater 
        response to the current error but possible overshooting and oscillation
        """

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

        # Wait for the topics to run
        time.sleep(1)

    def get_callback_compass(self):
        def _callback_compass(msg):
            """Get the compass heading from /auv/devices/compass topic"""
            self.compass = msg.data

        def _callback_compass_dvl(msg):
            """Get the compass heading from dvl"""
            self.compass = msg.data
            self.dvl.compass_rad = math.radians(msg.data)

        # If DVL return compass data from the DVL, else from the compass
        if self.dvl:
            return _callback_compass_dvl
        else:
            return _callback_compass

    def callback_depth(self, msg):
        """Get depth data from barometer /auv/devices/baro topic"""
        self.depth = msg.data[0]

    def set_depth(self, d):
        """Set depth to a given absolute value"""
        depth = Float64()
        depth.data = d
        self.pub_depth.publish(depth)
        print(f"[INFO] Depth set to {d}")

    def set_relative_depth(self, delta_depth):
        """Change the depth of the sub by a relative value (up 3 meters, down 3 meters, etc.)"""
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
        Move the robot in a given direction, by directly changing the PWM value of each thruster. This does not take input from the DVL.
        This is a non-blocking function.
        Inputs are between -5 and 5

        Args:
            yaw (float): Power for the yaw maneuver
            forward (float): Power to move forward
            lateral (float): Power for moving laterally (negative one way (less than 1500), positive the other way (more than 1500))
            pitch (float): Power for the pitch maneuver
            roll (float): Power for the roll maneuver
            vertical (float): Distance to change the depth by

        # TODO Handle timeout of the pixhawk
        """

        pwm = mavros_msgs.msg.OverrideRCIn()

        # Calculate PWM values
        channels = [1500] * 18
        # channels[2] = int((vertical * 80) + 1500) if vertical else 1500
        channels[3] = int((yaw * 80) + 1500) if yaw else 1500
        channels[4] = int((forward * 80) + 1500) if forward else 1500
        channels[5] = int((lateral * 80) + 1500) if lateral else 1500
        channels[6] = int((pitch * 80) + 1500) if pitch else 1500
        channels[7] = int((roll * 80) + 1500) if roll else 1500
        pwm.channels = channels

        # Publish PWMs to /auv/devices/thrusters
        if vertical!=0: self.set_relative_depth(vertical)
        self.pub_thrusters.publish(pwm)

    def set_heading(self, target: int):
        """
        Yaw to the target heading; target heading is absolute (not relative)
        This is a blocking function
        
        Args:
            target (int): Absolute desired heading 
        """

        # Mod the target to make sure it is between 0 - 359 degrees
        target = (target) % 360
        print(f"[INFO] Setting heading to {target}")

        while not rospy.is_shutdown():
            if self.compass is None:
                print("[WARN] Compass not ready")
                time.sleep(0.5)
                continue

            error = heading_error(self.compass, target)

            # Normalize error to the range -1 to 1 for the PID controller
            output = self.PIDs["yaw"](-error / 180)

            print(f"[DEBUG] Heading error: {error}, output: {output} {self.compass} {target}")

            if abs(error) <= 1:
                print("[INFO] Heading reached")
                break

            self.movement(yaw=output)
            time.sleep(0.1)

        print(f"[INFO] Finished setting heading to {target}")

    def setHeadingOld(self, target: int):
        """
        Yaws until the sub reaches desired heading
        This function is deprecated

        Args:
            target (int): Absolute (not relative) desired target heading
        """
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        target = (target) % 360

        # dir variable is the direction -- 1 is clockwise, -1 is counterclockwise
        while not rospy.is_shutdown():
            current = int(self.compass)
            print(current)
            dir = 1  # cw
            diff = abs(target - current)
            # Switch the direction of the yaw since going the other way will be faster (since there are 360 degrees in a circle)
            if diff >= 180:
                dir *= -1
            if current > target:
                dir *= -1
            if diff >= 180:
                diff = 360 - diff
            # If farther from desired heading, speed will increase, if close to desired heading, speed will decrease
            # Note how this has to be handled manually (there is no PID controller implemented in this function)
            if diff <= 10:
                speed = 55
            else:
                speed = 70
            # Once compass is within 2 degrees of desired heading, publish PWMs to stop yawing
            if diff <= 1:
                pwm.channels[3] = 1500
                self.pub_thrusters.publish(pwm)  # Publishing pwms to stop yawing
                break
            else:
                pwm.channels[3] = 1500 + (dir * speed)
                self.pub_thrusters.publish(pwm)  # Publishing pwms to continue yawing

    def navigate_dvl(self, x, y, z, end_heading=None, relative_coord=True, relative_heading=True, update_freq=10):
        """
        To navigate using the DVL to a specific point. This includes 3-D mobility (forward, lateral, depth), not just 1-D (forward or backward). Since this method is complex 
        and requires the compass to work perfectly, if we want to move forward and only move forward or backward, use forward_dvl instead. The compass
        should be calibrated in order to run.

        This is a blocking function

        Args:
            x (float): distance in meters to move laterally
            y (float): distance in meters to move forward by
            z (depth): depth to move to

            **x and y are by default relative to the current position and heading; z on the other hand is absolute

            end_heading (optional, int): the heading to reach at the end of the navigation. It defaults to the heading necessary to reach the target point 
            from the starting point in a straight line

            relative_coord (bool): flag indicating whether the coordinates are relative or not
            relative_heading (bool): flag indicating whether the coordinates should be rotated by the relative heading
            update_freq (int): the frequency at which the PID controllers are updated (in Hz)
        """

        if self.dvl is None:
            print("[ERROR] DVL not available, cannot navigate")
            return

        # Reset the PID integrals (this resets Proportional, Integral, and Derivative controllers)
        for pid in self.PIDs.values():
            pid.reset()

        if not relative_coord:
            # Heading 0 is north, so the positive side of the y-axis is going towards the north
            # Change position of target coordinates to coordinates relative to the current position of the sub
            x -= self.dvl.position[0]
            y -= self.dvl.position[1]

        if relative_heading:
            # Rotate the vector [x, y] by the current heading (to make the heading relative)
            x, y = rotate_vector(x, y, self.compass)

        # Get the setpoint (target) heading from the relative coordinates
        target_heading = get_heading_from_coords(x, y)
        print(f"[INFO] Navigating to {x}, {y}, {z}, {target_heading}deg current {self.compass}")

        # Rotate to the heading and set the depth of the sub
        self.set_heading(target_heading)
        self.set_depth(z)

        # Enter a local scope to handle the coordinates cleanly
        with self.dvl:
            # Set the depth independently
            self.set_depth(z)

            # Navigate to the target point 
            while not rospy.is_shutdown():
                if not self.dvl.is_valid:
                    print("[WARN] DVL data not valid, skipping")
                    time.sleep(0.5)
                    continue

                # Ensure the position data is avaliable
                if not self.dvl.data_available:
                    continue
                self.dvl.data_available = False

                # Get the x and y error by rotating the vector by finding the difference between the x and y coordinates of the current position and the target position
                # Does this by rotating the vector [x_error, y_error] by the current heading (the function inv_rotate_vector() is in utils.py)
                err_x, err_y = inv_rotate_vector(
                    x - self.dvl.position[0],
                    y - self.dvl.position[1],
                    self.compass,
                )

                # Check if the target coordinates have been reached
                x_err_th = 0.1 + self.dvl.error[0]
                y_err_th = 0.1 + self.dvl.error[1]
                if abs(err_x) <= x_err_th and abs(err_y) <= y_err_th:
                    print("[INFO] Target reached")
                    break

                # Calculate PID outputs
                output_x = self.PIDs["lateral"](-err_x)
                output_y = self.PIDs["forward"](-err_y)
                print(f"[DEBUG] err_x={err_x}, err_y={err_y}, output_x={output_x}, output_y={output_y}")
                self.movement(lateral=output_x, forward=output_y)

    def forward_dvl(self, throttle, distance):
        """
        Move forward using the DVL.
        This is a blocking function.

        Args:
            throttle (float): power at which to move forward at
            distance (float): distance in meters to move forward by
        """
        if self.dvl is None:
            print("[ERROR] DVL not available, cannot navigate")
            return

        print(f"[INFO] Moving forward {distance}m at throttle {throttle}")

        # Enter a local scope to handle coordinates cleanly
        with self.dvl:
            # Navigate to the target point
            while not rospy.is_shutdown():
                if not self.dvl.is_valid:
                    print("[WARN] DVL data not valid, skipping")
                    time.sleep(0.5)
                    continue

                # Ensure position data is updated/avaliable
                if not self.dvl.data_available:
                    continue
                self.dvl.data_available = False

                # Find the y-axis error (recall that the y-axis is the forward-backwards dimension)
                y = self.dvl.position[1]
                error = distance - y

                # Check if the target has been reached
                if abs(error) <= 0.1:
                    print("[INFO] Target reached")
                    break
                
                # Apply gain to the error and clip by the maximum throttle value(s)
                forward_output = np.clip(error * 4, -throttle, throttle)
                print(f"[DEBUG] error={error}, forward_output={forward_output}")

                # Move forward using the PWM calculations in the movement function
                self.movement(forward=forward_output)

    def lateral_dvl(self, throttle, distance):
        """
        Move laterally using the DVL -- this contains the exact same method as forward_dvl except the x, not y-axis is used.
        This is a blocking function.

        Args:
            throttle (float): power to move at
            distance (float): distance to move laterally (in meters)
        """
        if self.dvl is None:
            print("[ERROR] DVL not available, cannot navigate")
            return

        print(f"[INFO] Moving laterally {distance}m at throttle {throttle}")

        # Enter a local scope to handle coordinates nicely
        with self.dvl:
            # Navigate to the target point
            while not rospy.is_shutdown():
                if not self.dvl.is_valid:
                    print("[WARN] DVL data not valid, skipping")
                    time.sleep(0.5)
                    continue

                # Ensure position data is updated/avaliable
                if not self.dvl.data_available:
                    continue
                self.dvl.data_available = False

                # Calculate the error between the current and target x-coordinates (recall that the x-axis is the lateral dimension)
                x = self.dvl.position[0]
                error = distance - x

                # Check if we reached the target
                if abs(error) <= 0.1:
                    print("[INFO] Target reached")
                    break
                
                # Apply gain and clip at the maximum throttle value(s)
                lateral_output = np.clip(error * 4, -throttle, throttle)
                print(f"[DEBUG] error={error}, lateral_output={lateral_output}")

                # Move laterally using PWM values
                self.movement(lateral=lateral_output)

    def forwardHeading(self, power, t):
        """
        Moves forward at a specific power for a duration of time, with a backstop method (method for gradually decreasing the speed of a vehicle to allow precise movements).

        Args:
            power (float): the power to move at
            t (float): duration of movement (in seconds)

        Equations for power levels:
            Power 1: 7.8t + 3.4 (in inches)
            Power 2: 21t + 0.00952
            Power 3: 32.1t - 18.7
        """
        
        forwardPower = (power * 80) + 1500  # Calculating PWM values for forward thrusters
        if t > 3:  # Designating time for the backstop so that inertia does not increase the distance travelled by the sub
            timeStop = t / 6
        else:
            timeStop = 0.5

        # t = t + timeStop # this doesn't make sense because this increases time over long distance

        # Power to stop the sub's forward momentum (less than 1500 is reverse)
        powerStop = 1500 - (power * 40)

        # Create and publish PWM values for moving forward
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[4] = forwardPower

        # For the amount of time specified, move forward by publishing PWMs continuously to the thrusters
        startTime = time.time()
        while time.time() - startTime < t:
            self.pub_thrusters.publish(pwm)  # Publishing PWMs for forward commands to thrusters
            time.sleep(0.1)

        print("[INFO] finished forward")
        # Calculating gradual decrease of thruster power
        gradDec = int((forwardPower - powerStop) / (timeStop * 10))
        startTime = time.time()
        # Gradually decreasing forward thruster power (PWMs) for the designated time calculated above
        while time.time() - startTime < timeStop:
            current_p = pwm.channels[4]
            pwm.channels[4] = current_p - gradDec
            self.pub_thrusters.publish(pwm)  # Publishing reduced PWMs for continued reduced forward thrust
            time.sleep(0.1)

        t2 = 0
        print("[INFO] finished backstopping")
        # Create neutral channels (1500 is the neutral value)
        pwm.channels = [1500] * 18
        startTime = time.time()
        # Publishing idle PWMs for 0.5 secs to stop the sub
        while time.time() - startTime <= 0.5:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.05)
        print("[INFO] finished forward heading")

    def lateralUni(self, power, t):
        """
        Calls the universal backstop function for lateral movement

        Args:
            power (float): power for the movement
            t (float): duration of the movement (in s)
        """
        forwardPower = (power * 80) + 1500
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[5] = int(forwardPower)
        startTime = time.time()
        while time.time() - startTime < t:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.1)
        self.backStop(pwm, t) # Universal backstop

    def forwardUni(self, power, t):
        """
        Calls the universal backstop function for forward movement

        Args:
            power (float): power for the movement
            t (float): duration of the movement (in s)
        """
        forwardPower = (power * 80) + 1500
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[4] = int(forwardPower)

        # If the sub is Graey calculate the PWM differently (go to channel 5 instead of 4 and subtract by power)
        if config.get("sub", "onyx") == "graey":
            pwm.channels[5] = 1500-int(power*7)
        startTime = time.time()
        while time.time() - startTime < t:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.1)
        self.backStop(pwm, t) # Universal backstop

    def mapping(self, x, in_min, in_max, out_min, out_max):
        """
        Map a value from one range onto another

        Args:
            x (float): value to be mapped
            in_min (float): minimum value of the input range
            in_max (float): maximum value of the input range
            out_min (float): minimum value of the output range
            out_max (float): maximum value of the output range
        
        Returns:   
            float: mapped value
        """
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    

    def backStop(self, pwm, t):
        """
        Universal backstop mechanism that is given a PWM value to reduce and overall time of movement which calculates
        inertia and sends negative PWM commands to the thrusters for reverse thrust to stop the sub.

        Args:
            pwm (int): the PWM value
            t (float): the duration of the movement (not for the backstop, but for the total movement)
        """
        # Designate the time for backstopping
        if t > 3:
            timeStop = t / 6
        else:
            timeStop = 0.5
        
        # Initialize neutral channels
        maxPower = [1500] * 18
        powerStop = [1500] * 18

        # Calculate maximum power and power to stop for each channel
        for i in range(len(pwm.channels)):
            if pwm.channels[i] != 1500:
                maxPower[i] = pwm.channels[i] # Don't go higher than the current value
                powerStop[i] = (1500 - pwm.channels[i]) / 2 + 1500 # To stop, cut the gain (current movement value - neutral) from neutral in half and reverse it (for reverse thrust)
        
        # Gradually decrease PWMs to stop the sub
        startTime = time.time()
        while time.time() - startTime <= timeStop:
            for i in range(len(pwm.channels)):
                # Map the time left from the range of the time calculated for backstopping onto the current PWM movement value to PWM value to reverse thrust
                # As the time duration for the backstop goes up, the amount of reverse thrust will increase (PWM will go down)
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
        
        # Set at neutral PWMs to stop the sub completely
        pwm.channels = [1500] * 18
        startTime = time.time()
        while time.time() - startTime <= 0.5:
            self.pub_thrusters.publish(pwm)
            time.sleep(0.05)
        print("[INFO] finished backstopping")

    # uses functions from testing to correlate pwms and time to distance and then utilizes forwardHeadingUni commmand to send pwms to thrusters for calculated pwms and time
    def forwardDist(self, dist, power):
        """
        Move the sub forward by a certain distance at a certain power -- uses functions created through testing to correlate the PWM value
        and time necessary to move a certain distance, then calls ForwardUni() to move that distance.

        Args:
            dist (float): distance to move forward (in meters)
            power (float): power level for forward movement 
        """
        # Convert distance from meters to inches
        inches = 39.37 * dist
        print(power, inches)
        eqPower = abs(round(power))
        time = 0
        # Calculate the amount of time needed based on the power level
        if eqPower >= 3:
            inches = inches - 9.843
            time = (inches + 18.7) / 32.1
        elif eqPower == 2:
            time = (inches - 0.01) / 21
        elif eqPower == 1:
            time = (inches - 3.4) / 7.8
        # Move forward for the specified time and at the specified power
        self.forwardUni(power, time)
