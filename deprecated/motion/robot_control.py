import signal
import threading
import time

import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import rospy
import sensor_msgs.msg
from std_msgs.msg import Float64, Int32MultiArray


class RobotControl:
    # initialize AUV state to idle (1500) and get initial compass heading
    def __init__(self):
        self.compass = None
        # establishing thrusters and depth publishers
        self.pubThrusters = rospy.Publisher(
            "auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn, queue_size=10
        )
        self.pubDepth = rospy.Publisher("auv/devices/setDepth", Float64, queue_size=10)
        rospy.init_node(
            "robotcontrol", anonymous=True
        )  # initializing robotcontrol node
        self.channels = [
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
        ]
        # initializing compass

        # beginning separate threading for compass sensor readings
        self.thread_compass = threading.Timer(0, self.get_compass)
        self.thread_compass.daemon = True
        self.thread_compass.start()
        print("RC Lib Initialized")

        # stores previously set depth
        self.previous_depth = 0.0

    # Constantly loop for checking compass values from /auv/devices/compass
    def get_compass(self):
        print("get compass running..")
        rospy.Subscriber("/auv/devices/compass", Float64, self.compass_cb)
        rospy.spin()  # infinitely checking compass reading until program has ended

    # Retrieves current compass heading from mavros output to rostopic /auv/devices/compass
    def compass_cb(self, data):
        self.compass = data.data  # assigns self.compass current heading

    # have not used mapping at all yet, possibly for localization
    def mapping(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # returns compass heading
    def return_compass(self):
        return self.compass

    # setting depth for depth holding to specific input
    def setDepth(self, d):
        depth = Float64()
        depth.data = d
        for i in range(5):
            # publishing depth values to /auv/devices/setDepth where it will depth hold to given value
            self.pubDepth.publish(depth)
            time.sleep(0.1)
        print("successfully set depth")
        self.previous_depth = d

    def movement(
        self,
        throttle=None,
        forward=None,
        lateral=None,
        yaw=None,
        pitch=None,
        roll=None,
    ):
        # provided an kwargs of values from -5 to 5 for each degree of freedom
        pwm = mavros_msgs.msg.OverrideRCIn()
        channels = self.channels
        # translating -5 to 5 to 1100-1900 pwm values
        if throttle:
            channels[2] = (throttle * 80) + 1500
        if forward:
            channels[4] = (lateral * 80) + 1500
        if lateral:
            channels[5] = (lateral * 80) + 1500
        if yaw:
            channels[3] = (yaw * 80) + 1500
        if pitch:
            channels[6] = (pitch * 80) + 1500
        if roll:
            channels[7] = (roll * 80) + 1500
        pwm.channels = channels
        # publishing pwms to /auv/devices/thrusters; sending commands to thrusters
        self.pubThrusters.publish(pwm)

    # set heading is a function that yaws until reaches desired heading input integer
    def setHeading(self, target: int):
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
                self.pubThrusters.publish(pwm)  # publishing pwms to stop yawing
                break
            else:
                pwm.channels[3] = 1500 + (dir * speed)
                self.pubThrusters.publish(pwm)  # publishing pwms to continue yawing

        print("Heading is set")

    def forwardHeading(self, power, t):
        # Power 1: 7.8t+3.4 (in inches)
        # Power 2: 21t+0.00952
        # Power 3: 32.1t-18.7
        forwardPower = (power * 80) + 1500  # calculating power for forward thrusters
        if (
            t > 3
        ):  # designating time for backstopping function to prevent inertia from increasing distance travelled
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
            self.pubThrusters.publish(
                pwm
            )  # publishing pwms for forward commands to thrusters
            time.sleep(0.1)

        print("finished forward")
        # calculating gradual decrease for thruster power
        gradDec = int((forwardPower - powerStop) / (timeStop * 10))
        startTime = time.time()
        # gradually decreasing forward thruster power for designated time calculated above
        while time.time() - startTime < timeStop:
            current_p = pwm.channels[4]
            pwm.channels[4] = current_p - gradDec
            self.pubThrusters.publish(pwm)  # publishing reduced pwms for forward thrust
            time.sleep(0.1)

        t2 = 0
        print("finished backstopping")
        pwm.channels = [1500] * 18
        startTime = time.time()
        # publishing idle pwms to sub to stop moving
        while time.time() - startTime <= 0.5:
            self.pubThrusters.publish(pwm)
            time.sleep(0.05)
        print("finished forward heading")

    # incorporates universal backstop function for lateral movement
    def lateralUni(self, power, t):
        forwardPower = (power * 80) + 1500
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[5] = forwardPower
        startTime = time.time()
        while time.time() - startTime < t:
            self.pubThrusters.publish(pwm)
            time.sleep(0.1)
        self.backStop(pwm, t)

    # incorporates universal backstop for forward movement
    def forwardHeadingUni(self, power, t):
        forwardPower = (power * 80) + 1500
        pwm = mavros_msgs.msg.OverrideRCIn()
        pwm.channels = [1500] * 18
        pwm.channels[4] = forwardPower
        startTime = time.time()
        while time.time() - startTime < t:
            self.pubThrusters.publish(pwm)
            time.sleep(0.1)
        self.backStop(pwm, t)

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
            self.pubThrusters.publish(pwm)
            time.sleep(0.05)
        pwm.channels = [1500] * 18
        startTime = time.time()
        while time.time() - startTime <= 0.5:
            self.pubThrusters.publish(pwm)
            time.sleep(0.05)
        print("finished backstopping")

    # uses functions from testing to correlate pwms and time to distance and then utilizes forwardHeadingUni commmand to send pwms to thrusters for calculated pwms and time
    def forwardDist(self, dist, power):
        inches = 39.37 * dist
        print(power, inches)
        if power == 3:
            inches = inches - 9.843
            time = (inches + 18.7) / 32.1
            print(time)
            self.forwardHeadingUni(3, time)
        elif power == 2:
            time = (inches - 0.01) / 21
            self.forwardHeadingUni(2, time)
        elif power == 1:
            time = (inches - 3.4) / 7.8
            self.forwardHeadingUni(1, time)
        print("completed forward with distance!")

    def onExit(signum, frame):
        try:
            print("\nDisarming and exiting...")
            rospy.signal_shutdown("Rospy Exited")
            while not rospy.is_shutdown():
                pass
            print("\n\nCleanly Exited")
            exit(1)
        except:
            pass

    signal.signal(signal.SIGINT, onExit)
