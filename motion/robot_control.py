import rospy
import time
import mavros_msgs.msg
import mavros_msgs.srv
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64
import sensor_msgs.msg
import geometry_msgs.msg


class RobotControl:
    def __init__(self):
        self.pub = rospy.Publisher('auv/devices/thrusters', mavros_msgs.msg.OverrideRCIn, queue_size=10)
        rospy.init_node("robotcontrol", anonymous=True)
        self.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        print("RC Lib Initialized")
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/auv/devices/compass", Float64, compass_cb)
        
    def compass_cb(data):
        self.compass = data
        
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
        
    def setHeading(self, target: int):
        minTarget = ((target-2) + 360) % 360
        maxTarget = (target+2) % 360
        pwm = mavros_msgs.msg.OverrideRCIn()
        rate = rospy.Rate(5)
        pwm.channels = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        while True:
            current = self.compass
            print(current)
            dir = 1  # cw
            diff = abs(target-current)
            if(diff >= 180):
                dir *= -1
            if(current > target):
                dir *= -1
            if(diff >= 180):
                diff = 360-diff
            if(diff <= 10):
                speed = 100
            else:
                speed = 200
            if(diff <= 2):
                pwm.channels[3] = 1500
                self.pub.publish(pwm)
                break
            else:
                pwm.channels[3] = 1500+(dir*speed)
                self.pub.publish(pwm)
          
        print("Heading is set")
                
#       def forwardHeading(self, power, time):
#             deg = self.compass
#             forwardPower = (power*80)+1500
#             t=0;
#             pwm = mavros_msgs.msg.OverrideRCIn()
#             rate = rospy.Rate(5)
#             pwm.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
#             print(pwm.channels)
#             self.pub.publish(pwm)
#             while True:
#                 new_deg = self.compass
#                 if(abs(new_deg-deg)>2):
#                     setHeading(deg)
#                 time.sleep(1)
#                 t=t+1
#                 if(t=time):
#                     break
     
