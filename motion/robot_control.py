import rospy
import time
import mavros_msgs.msg
import mavros_msgs.srv
from std_msgs.msg import Int32MultiArray
import sensor_msgs.msg
import geometry_msgs.msg


class RobotControl:
    def __init__(self):
        self.pub = rospy.Publisher('auv/devices/thrusters', mavros_msgs.msg.OverrideRCIn, queue_size=10)
        rospy.init_node("robotcontrol", anonymous=True)
        self.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        print("RC Lib Initialized")
        
    def movement(self, **array):
        pwm = mavros_msgs.msg.OverrideRCIn()
        rate = rospy.Rate(5)
        channels = self.channels
        channels[2] = ((array["throttle"]*80) + 1500)
        channels[4] = ((array["forward"]*80) + 1500)
        channels[5] = ((array["lateral"]*80) + 1500)
        channels[3] = ((array["yaw"]*80) + 1500)
        channels[6] = ((array["pitch"]*80) + 1500)
        channels[7] = ((array["roll"]*80) + 1500)
        pwm.channels = channels
        print(pwm.channels)
        rate.sleep()
        self.pub.publish(pwm)
        time.sleep(array["t"])
        pwm.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        print(pwm.channels)
        self.pub.publish(pwm)
