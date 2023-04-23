import mavros_client as mc
from std_msgs.msg import Int32MultiArray

sd = {
	"mode": [],
	"state": []
}

def pwm_cb(p): 
	if sd['mode'] == "raw":
		mc.channels = p.data

subscribers = [
	["mode", "/auv/motion/mode", String, mode_cb, NULL],
	["raw", "/auv/motion/raw", Int32MultiArray, pwm_cb, NULL],
]

def init_ros_io(p, s):
	for i in s:
                i[4] = rospy.Subscriber(i[1], i[2], i[3], queue_size=10)

        for i in p:
		i[3] = rospy.Publisher(i[1], i[2], queue_size=10)
 

def main():
	rospy.init_node("motion_handler", anonymous=True)
	mc.init_ros_io(mc.publishers, mc.subscribers)
	init_ros_io([], subscribers)

	mc.set_mode("ALT_HOLD")
	mc.connect_arm()
	mc.send_rc()
	
	rospy.spin()
