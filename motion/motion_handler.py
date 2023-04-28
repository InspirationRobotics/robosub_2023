import rospy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

def get_pub(n, p):
        return p[n][2]

publishers = {
        "thrusters": ["/auv/devices/thrusters", Int32MultiArray, None]
}

sd = {
	"mode": [],
}

def pwm_cb(p): 
#	if sd['mode'] == "raw":
        get_pub("thrusters", publishers).publish(p)

def mode_cb(p):
        pass

subscribers = [
	["mode", "/auv/motion/mode", String, mode_cb, NULL],
	["raw", "/auv/motion/raw", Int32MultiArray, pwm_cb, NULL], # basically just passthrough
]

def init_ros_io(p, s):
        for i in s:
                i[4] = rospy.Subscriber(i[1], i[2], i[3], queue_size=10)
                
        for i in p:
                i[3] = rospy.Publisher(i[1], i[2], queue_size=10)
 

def main():
	rospy.init_node("motion_handler", anonymous=True)
	init_ros_io(publishers, subscribers)
	rospy.spin()
