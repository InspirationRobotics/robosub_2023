# $ rosrun mavros mavparam set SYSID_MYGCS 1

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import OverrideRCIn, State
import threading
import time

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def msg_send():
    last_req = rospy.Time.now()
    
    while(not rospy.is_shutdown()):
        if(current_state.mode != "MANUAL" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(stab_set_mode).mode_sent == True):
                rospy.loginfo("stabilize enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
                
        rc_pub.publish(msg)

if __name__ == "__main__":
    values = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
    
    msg = OverrideRCIn()
    msg.channels = values
    
    rospy.init_node("mavros_example_client")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rc_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)
                                 
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    
    stab_set_mode = SetModeRequest()
    stab_set_mode.custom_mode = 'MANUAL'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    

    thread = threading.Thread(target=msg_send)
    thread.start()


    i = 0
    while True:
        if i % 2 == 0:
            values[3] = 1500
        else:
            values[3] = 1550
        time.sleep(0.2)
        i += 1
