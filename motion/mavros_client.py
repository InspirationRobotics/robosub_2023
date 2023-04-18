# $ rosrun mavros mavparam set SYSID_MYGCS 1

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import OverrideRCIn, State
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import threading
import time

rc_pub = None
rate = None
state_sub = None
current_state = State()
mode = "STABILIZE"

arming_client = None
set_mode_client = None
stab_set_mode = None
arm_cmd = None

compass = None
imu = None

channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

def state_cb(msg):
    global current_state
    current_state = msg

def compass_cb(msg):
    global compass
    compass = msg.data
    print(compass) 

def imu_cb(msg):
    global imu
    imu = msg
    
def init_ros():
    global rc_pub
    global rate
    global state_sub

    rospy.init_node("mavros_example_client")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    compass_sub = rospy.Subscriber("mavros/global_position/compass_hdg", Float64, callback = compass_cb)
    rc_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)
                                 
    rate = rospy.Rate(20)

def set_mode(m):
	global mode
	mode = m

def connect_arm():
    global arming_client
    global set_mode_client
    global stab_set_mode
    global arm_cmd
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    stab_set_mode = SetModeRequest()
    stab_set_mode.custom_mode = mode
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

def send_rc():
    msg = OverrideRCIn()
    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != mode and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(stab_set_mode).mode_sent == True):
                rospy.loginfo("mode enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        msg.channels = channels
        rc_pub.publish(msg)

def getIMU():
    pass
    
def getCompass():
    return compass

