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
        self.pubThrusters = rospy.Publisher('auv/devices/thrusters', mavros_msgs.msg.OverrideRCIn, queue_size=10)
        self.pubDepth = rospy.Publisher('auv/devices/setDepth', Float64, queue_size=10)
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
    
    def mapping(x, in_min, in_max, out_min, out_max): 
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def return_compass(self):
        return self.compass

    def setDepth(self, d):
        depth = Float64()
        depth.data = d
        timer=0
        print(depth)
        while(timer<1):
            time.sleep(0.1)
            self.pubDepth.publish(depth)
            timer=timer+0.1
            
        print("successfully set depth")
        
    def movement(self, **array):
        pwm = mavros_msgs.msg.OverrideRCIn()
        channels = self.channels
        channels[2] = ((array["throttle"]*80) + 1500)
        channels[4] = ((array["forward"]*80) + 1500)
        channels[5] = ((array["lateral"]*80) + 1500)
        channels[3] = ((array["yaw"]*80) + 1500)
        channels[6] = ((array["pitch"]*80) + 1500)
        channels[7] = ((array["roll"]*80) + 1500)
        pwm.channels = channels       
        self.pubThrusters.publish(pwm)
        #print(pwm.channels)
        
    def setHeading(self, target: int):
        pwm = mavros_msgs.msg.OverrideRCIn()
        rate = rospy.Rate(5)
        pwm.channels = [1500]*18
        target = ((target)%360)
        while True:
            current = self.compass
            current = int(current)
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
                speed = 55
            else:
                speed = 70
            if(diff <= 2):
                pwm.channels[3] = 1500
                print(pwm.channels)
                self.pubThrusters.publish(pwm)
                break
            else:
                pwm.channels[3] = 1500+(dir*speed)
                print(pwm.channels)
                self.pubThrusters.publish(pwm)
          
        print("Heading is set")
                
    def forwardHeading(self, power, t):
        #Power 1: 7.8t+3.4 (in inches)
        #Power 2: 21t+0.00952
        #Power 3: 32.1t-18.7
        forwardPower = (power*80)+1500
        timer=0
        if (t>3):
            timeStop = t/6
        else:
            timeStop = 0.5
        
        t = t+timeStop
        powerStop = 1500 - (power*40)
        print(timeStop, powerStop)
        pwm = mavros_msgs.msg.OverrideRCIn()
        rate = rospy.Rate(5)
        pwm.channels = [1500]*18
        pwm.channels[4] = forwardPower
        print(pwm)
        startTime = time.time()
        while (time.time()-startTime<t):
            self.pubThrusters.publish(pwm)
            time.sleep(0.1)

        print("finished forward")
        gradDec = int((forwardPower-powerStop)/(timeStop*10))
        startTime = time.time()
        while (time.time()-startTime<timeStop):
            current_p = pwm.channels[4]
            pwm.channels[4] = current_p - gradDec
            print(pwm.channels)
            self.pubThrusters.publish(pwm)
            time.sleep(0.1)
        
        t2=0
        print("finished backstopping")
        pwm.channels = [1500]*18
        while (t2 <= 0.5):
            self.pubThrusters.publish(pwm)
            time.sleep(0.1)
            t2=t2+0.1
        print(pwm)
        print("finished forward heading")

    def forwardDist(self, dist, power):
        inches = 39.37*dist
        print(power, inches)
        if(power==3):
            inches = inches-9.843
            time = (inches+18.7)/32.1
            print(time)
            self.forwardHeading(3, time)
        elif(power==2):
            time = (inches-0.01)/21
            self.forwardHeading(2, time)
        elif(power==1):
            time=(inches-3.4)/7.8
            self.forwardHeading(1, time)
        
        print("completed forward with distance!")

