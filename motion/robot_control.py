import rospy
import time
import mavros_msgs.msg
import mavros_msgs.srv
import geographic_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg


class RobotControl:
    def __init__(self):
        self.pub = rospy.Publisher('auv/devices/thrusters', mavros_msgs.msg.OverrideRCIn, queue_size=10)
        rospy.init_node("robotcontrol", anonymous=True)
        self.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        print("RC Lib Initialized")
        
    def movement(self, throttle, forward, lateral, yaw, pitch, roll, time):
        pwm = mavros_msgs.msg.OverrideRCIn()
        rate = rospy.Rate(5)
        channels = self.channels
        channels[2] = ((throttle*80) + 1500)
        channels[4] = ((forward*80) + 1500)
        channels[5] = ((lateral*80) + 1500)
        channels[3] = ((yaw*80) + 1500)
        channels[6] = ((pitch*80) + 1500)
        channels[7] = ((roll*80) + 1500)
        pwm.channels = channels
        rate.sleep()
        self.pub.publish(pwm)
        time.sleep(int(time))
        self.pub.publish(self.channels)
        
        
