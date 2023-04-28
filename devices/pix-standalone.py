# this will create the interfaces to the pixhawk, including the pixhawk barometer (r), pix IMU (r), pixhawk compass (r), thrusters (w)
# need to run mavros first
# note: to enable manually overriding RC, run `rosrun mavros mavparam set SYSID_MYGCS1`

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import OverrideRCIn, State
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import threading
import time

channels = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

pix_status = {
    "armed": False,
    "arming_client": None,
    "set_mode_client": None,
    "stab_set_mode": None,
    "arm_cmd": None
}

def get_pub(n, p):
    return p[n][2]

publishers = {
    "compass": ["/auv/devices/compass", Float64, None],
    "imu": ["/auv/devices/imu", Imu, None],
    "baro": ["/auv/devices/baro", FluidPressure, None],
    "thrusters": ["/mavros/rc/override", OverrideRCIn, None]
}

sub_data = {
    # to be stored
    "state": None
}

def state_cb(msg):
    sub_data["state"] = msg

def compass_cb(msg):
    get_pub("compass", publishers).publish(msg.data)

def imu_cb(msg):
    get_pub("imu", publishers).publish(msg)

def baro_cb(msg):
    get_pub("imu", publishers).publish(msg)

def thruster_cb(msg):
    rc_msg = OverrideRCIn()
    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        if(sub_data["state"].mode != mode and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(stab_set_mode).mode_sent == True):
                rospy.loginfo("mode enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not sub_data["state"].armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(sub_data["arm_cmd"]).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        rc_msg.channels = msg.data
        get_pub("thrusters", publishers).publish(rc_msg)
    
subscribers = {
    "state": ["/mavros/state", State, state_cb, None],
    "compass": ["/mavros/global_position/compass_hdg", Float64, compass_cb, None],
    "imu": ["/mavros/imu/data", Imu, imu_cb, None],
    "baro": ["/mavros/imu/FluidPressure", FluidPressure, baro_cb, None],
    "thrusters": ["/auv/devices/thrusters", Int32MultiArray, thruster_cb, None]
}

def init_ros_io(p, s):
    for i in p:
        i[2] = rospy.Publisher(i[0], i[1], queue_size=10)

    for i in s:
        i[3] = rospy.Subscriber(i[0], i[1], i[2], queue_size=10)

def connect_arm():
    rospy.wait_for_service("/mavros/cmd/arming")
    pix_status["arming_client"] = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/mavros/set_mode")
    pix_status["set_mode_client"] = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    while(not rospy.is_shutdown() and not sub_data["state"].connected):
        rate.sleep()

    pix_status["stab_set_mode"] = SetModeRequest()
    pix_status["stab_set_mode"].custom_mode = mode
    pix_status["arm_cmd"] = CommandBoolRequest()
    pix_status["arm_cmd"].value = True

def main():
    rospy.init_node("pix_interface", anonymous=True)
    init_ros_io(publishers, subscribers)
    connect_arm()
    rospy.spin()

# main()
