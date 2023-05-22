
# really simple baseline mission
import rospy
import time
import mavros_msgs.msg
import mavros_msgs.srv
import geographic_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg

#from insp_msgs.msg import PointArray

def get_pub(n, p):
        pub = p[n][2]
        return pub

publishers = {
        #"raw": ["/auv/motion/raw", mavros_msgs.msg.OverrideRCIn, None]
        "raw": ["/auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn, None] #bypass motionhandler temporarily since it's just a passthrough
}

def pwm_cb(p): 
#	if sd['mode'] == "raw":
        get_pub("thrusters", publishers).publish(p)

def init_ros_io(p, s):
        if s is not None:
           for i in s.values():
               i[3] = rospy.Subscriber(i[0], i[1], i[2], queue_size=10)
        
        if p is not None:
           for i in p.values():
               i[2] = rospy.Publisher(i[0], i[1], queue_size=10)

def SDoF(th, fw, lat, yaw, pitch, roll):
    a = mavros_msgs.msg.OverrideRCIn()
    a.channels = [1500, 1500, th, yaw, fw, pitch, roll, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
    print(a.channels)
    get_pub("raw", publishers).publish(a)
                
def main():
    print("entered main")
    rospy.init_node("path_v1", anonymous=True)
    init_ros_io(publishers, None)
    print("init done")
    # SDoF(1500, 1500, 1500, 1600, 1500, 1500)
    SDoF(1500, 1500, 1500, 1500, 1500, 1500)
    time.sleep(2)
    SDoF(1500, 1700, 1500, 1500, 1500, 1500)
    print("finished executing square mission")

if __name__ == "__main__":
   main()
