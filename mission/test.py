# really simple baseline mission
import rospy
from std_msgs.msg import Int32MultiArray
#from insp_msgs.msg import PointArray


def get_pub(n, p):
        return p[n][2]

publishers = {
        "raw": ["/auv/motion/raw", Int32MultiArray, None]
}

def pwm_cb(p): 
#	if sd['mode'] == "raw":
        get_pub("thrusters", publishers).publish(p)

def init_ros_io(p, s):
        for i in s.values():
                i[3] = rospy.Subscriber(i[0], i[1], i[2], queue_size=10)
                
        for i in p.values():
                i[2] = rospy.Publisher(i[0], i[1], queue_size=10)

def SDoF(th, fw, lat, yaw, pitch, roll):
    get_pub("raw", publishers).publish([1500, 1500, th, yaw, fw, pitch, roll, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
                
def main():
    rospy.init_node("path_v1", anonymous=True)

    while not rospy.is_shutdown():
        SDof(1500, 1550, 1500, 1500, 1500, 1500)
        sleep(1)
    print("finished executing square mission")
    rospy.spin()
