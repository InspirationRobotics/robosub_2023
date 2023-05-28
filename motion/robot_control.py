import rospy
import time
import mavros_msgs.msg
import mavros_msgs.srv
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64
import sensor_msgs.msg
import geometry_msgs.msg
import threading

class RobotControl:

    def __init__(self):
        self.compass = None
        self.pub = rospy.Publisher('auv/devices/thrusters', mavros_msgs.msg.OverrideRCIn, queue_size=10)
        rospy.init_node("robotcontrol", anonymous=True)
        self.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        self.thread_compass = threading.Timer(0, self.get_compass)
        self.thread_compass.daemon = True
        self.thread_compass.start()
        print("RC Lib Initialized")
        
    def get_compass(self):
        print("get compass running..")
        rospy.Subscriber("/auv/devices/compass", Float64, self.compass_cb)
        rospy.spin()

    def compass_cb(self, data):
        self.compass = data.data

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
        timer=0
        while(timer<array["t"]):
            self.pub.publish(pwm)
            time.sleep(0.1)
            timer=timer+0.1
            #print(timer)

        #time.sleep(array["t"])
        pwm.channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        print(pwm.channels)
        self.pub.publish(pwm)
        
    def setHeading(self, target: int, error: int):
        print(target, error)
        minTarget = ((target-error) + 360) % 360
        maxTarget = (target+error) % 360
        pwm = mavros_msgs.msg.OverrideRCIn()
        rate = rospy.Rate(5)
        pwm.channels = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        while True:
            current = self.compass
            current = int(current)
            print(current)
            dir = 1  # cw
            diff = abs(target-current)
            print(diff)
            if(diff >= 180):
                dir *= -1
            if(current > target):
                dir *= -1
            if(diff >= 180):
                diff = 360-diff
            if(diff <= 10):
                speed = 25
            else:
                speed = 40
            if(diff <= error):
                pwm.channels[3] = 1500
                self.pub.publish(pwm)
                break
            else:
                pwm.channels[3] = 1500+(dir*speed)
                self.pub.publish(pwm)
          
        print("Heading is set")
                
    def forwardHeading(self, power, t):
        deg = self.compass
        forwardPower = (power*80)+1500
        print(forwardPower)
        t1=0
        pwm = mavros_msgs.msg.OverrideRCIn()
        rate = rospy.Rate(5)
        pwm.channels = [1500]*18
        pwm.channels[3] = forwardPower
        print(pwm.channels)
        self.pub.publish(pwm)
        while True:
         #   new_deg = self.compass
          #  if(abs(new_deg-deg)>2):
           #      setHeading(deg, 2)
            time.sleep(1)
            t1=t1+1
            if(t1==t):
                break
        pwm.channels[3] = 1500
        self.pub.publish(pwm)
