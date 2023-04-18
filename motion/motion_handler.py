import mavros_client as mc
from std_msgs.msg import Int32MultiArray

sd = {
	"mode": []
}

def pwm_cb(p): 
	if sd['mode'] == "raw":
		mc.channels = p.data

subscribers = [
		["/auv/motion/mode", String, mode_cb, NULL]
		["/auv/motion/raw", Int32MultiArray, pwm_cb, NULL]
]

init_ros(p, s, node_name):
	for i in s:
                i[3] = rospy.Subscriber(i[0], i[1], i[2], queue_size=10)

        rospy.init_node(node_name, anonymous=True)

def main():
	mc.init_ros()
	mc.set_mode("ALT_HOLD")
	mc.connect_arm()
	init_ros([], subscribers, "motion_handler")
	
	rospy.spin()



