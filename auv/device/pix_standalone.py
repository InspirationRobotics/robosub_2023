import lsb_release

if lsb_release.get_distro_information()["RELEASE"] == "18.04":
    import ctypes

    libgcc_s = ctypes.CDLL("libgcc_s.so.1")

import platform
import signal
import threading
import time
from statistics import mean
from struct import pack, unpack

import geographic_msgs.msg
import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import numpy as np
import rospy
import sensor_msgs.msg
import std_msgs.msg
from simple_pid import PID

from ..utils import statusLed
from ..utils.rospyHandler import RosHandler
from ..utils.topicService import TopicService

MODE_MANUAL = "MANUAL"
MODE_STABILIZE = "STABILIZE"
MODE_ALTHOLD = "ALT_HOLD"
MODE_LOITER = "LOITER"
MODE_AUTO = "AUTO"
MODE_GUIDED = "GUIDED"


class AUV(RosHandler):
    def __init__(self):
        super().__init__()
        self.armed = False
        self.guided = False
        self.DepthHoldMode = False
        self.depthMotorPower = 0
        self.depth = None
        self.mode = ""
        self.channels = [0] * 18
        self.depthCalib = 0
        self.sub = True  # grey
        self.limNeu = [200, 1490]  # grey
        if "nx" in platform.node():
            self.sub = False  # onyx
            self.limNeu = [300, 1480]  # onyx
        self.pid = PID(self.limNeu[0], 0.05, 0, setpoint=0.5)  # in meters
        self.pid.output_limits = (-self.limNeu[0], self.limNeu[0])

        # init topics
        self.TOPIC_STATE = TopicService("/mavros/state", mavros_msgs.msg.State)
        self.SERVICE_ARM = TopicService("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.SERVICE_SET_MODE = TopicService("/mavros/set_mode", mavros_msgs.srv.SetMode)
        self.SERVICE_SET_PARAM = TopicService("/mavros/param/set", mavros_msgs.srv.ParamSet)
        self.SERVICE_GET_PARAM = TopicService("/mavros/param/get", mavros_msgs.srv.ParamGet)

        # movement
        self.TOPIC_SET_VELOCITY = TopicService(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist
        )  # only works in auto/guided mode
        self.TOPIC_SET_RC_OVR = TopicService("/mavros/rc/override", mavros_msgs.msg.OverrideRCIn)

        # sensory
        self.TOPIC_GET_IMU_DATA = TopicService("/mavros/imu/data", sensor_msgs.msg.Imu)
        self.TOPIC_GET_CMP_HDG = TopicService("/mavros/global_position/compass_hdg", std_msgs.msg.Float64)
        self.TOPIC_GET_RC = TopicService("/mavros/rc/in", mavros_msgs.msg.RCIn)
        self.TOPIC_GET_MAVBARO = TopicService("/mavlink/from", mavros_msgs.msg.Mavlink)
        # https://discuss.bluerobotics.com/t/ros-support-for-bluerov2/1550/24
        self.TOPIC_GET_BATTERY = TopicService("/mavros/battery", sensor_msgs.msg.BatteryState)

        # custom topics
        self.AUV_COMPASS = TopicService("/auv/devices/compass", std_msgs.msg.Float64)
        self.AUV_IMU = TopicService("/auv/devices/imu", sensor_msgs.msg.Imu)
        self.AUV_BARO = TopicService("/auv/devices/baro", std_msgs.msg.Float32MultiArray)
        self.AUV_GET_THRUSTERS = TopicService("/auv/devices/thrusters", mavros_msgs.msg.OverrideRCIn)
        self.AUV_GET_DEPTH = TopicService("/auv/devices/setDepth", std_msgs.msg.Float64)
        self.AUV_GET_ARM = TopicService("/auv/status/arm", std_msgs.msg.Bool)
        self.AUV_GET_MODE = TopicService("/auv/status/mode", std_msgs.msg.String)

    def arm(self, status: bool):
        if status:
            statusLed.red(True)
        else:
            statusLed.red(False)
        data = mavros_msgs.srv.CommandBoolRequest()
        data.value = status
        self.SERVICE_ARM.set_data(data)
        result = self.service_caller(self.SERVICE_ARM, timeout=30)
        return result.success, result.result

    def get_param(self, param: str):
        data = mavros_msgs.srv.ParamGetRequest()
        data.param_id = param
        self.SERVICE_GET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_GET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def set_param(self, param: str, value_integer: int, value_real: float):
        data = mavros_msgs.srv.ParamSetRequest()
        data.param_id = param
        data.value.integer = value_integer
        data.value.real = value_real
        self.SERVICE_SET_PARAM.set_data(data)
        result = self.service_caller(self.SERVICE_SET_PARAM, timeout=30)
        return result.success, result.value.integer, result.value.real

    def change_mode(self, mode: str, flag=False):
        # TODO: see which mode we want to keep
        if mode == MODE_ALTHOLD:
            self.DepthHoldMode = True
            self.change_mode(MODE_STABILIZE, True)
            return
        self.DepthHoldMode = flag
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent

    def calibrateDepth(self):
        print("\nStarting Depth Calibration...")
        start_time = time.time()
        depthArr = []
        while self.depth == None:
            pass
        prevDepth = self.depth
        while time.time() - start_time < 3:
            if self.depth != prevDepth:
                depthArr.append(self.depth)
                prevDepth = self.depth
        self.depthCalib = mean(depthArr)
        print(f"Finished. Surface is: {self.depthCalib}")
        return

    def depthHold(self, depth):
        try:
            depth = depth - self.depthCalib
            if depth < -9 or depth > 100:
                return
            self.depthMotorPower = int(self.pid(depth) * -1 + self.limNeu[1])
            print(f"Depth: {depth:.4f} depthMotorPower: {self.depthMotorPower} Target: {self.pid.setpoint}")
            # assume motor range is 1200-1800 so +-300
        except Exception as e:
            print("DepthHold error")
            print(e)

    def get_baro(self, baro):
        try:
            if baro.msgid == 143:
                p = pack("QQ", *baro.payload64)
                time_boot_ms, self.press_abs, self.press_diff, temperature = unpack("Iffhxx", p)  # pressure is in mBar
                self.press_abs = round(self.press_abs, 2)
                self.depth = self.press_abs / (997.0474 * 9.80665 * 0.01)
                self.press_diff = round(self.press_diff, 2)
                baro_data = std_msgs.msg.Float32MultiArray()
                baro_data.data = [self.depth - self.depthCalib, self.press_diff]
                self.AUV_BARO.set_data(baro_data)
                self.topic_publisher(topic=self.AUV_BARO)
                if self.DepthHoldMode and self.armed:
                    self.depthHold(self.depth)
        except Exception as e:
            print("Baro Failed")
            print(e)

    def setDepth(self, setDValue):
        if setDValue.data < 0:
            return
        self.pid.setpoint = setDValue.data

    def batteryIndicator(self, msg):
        if self.sub:
            self.voltage = msg.voltage
            if self.voltage < 13.5:
                statusLed.flashRed()
        pass

    def enable_topics_for_read(self):
        self.topic_subscriber(self.TOPIC_STATE)
        self.topic_subscriber(self.TOPIC_GET_IMU_DATA)
        self.topic_subscriber(self.TOPIC_GET_CMP_HDG)
        self.topic_subscriber(self.TOPIC_GET_RC)
        self.topic_subscriber(self.AUV_GET_THRUSTERS)
        self.topic_subscriber(self.AUV_GET_ARM)
        self.topic_subscriber(self.AUV_GET_MODE)
        self.topic_subscriber(self.TOPIC_GET_MAVBARO, self.get_baro)
        self.topic_subscriber(self.AUV_GET_DEPTH, self.setDepth)
        self.topic_subscriber(self.TOPIC_GET_BATTERY, self.batteryIndicator)
        # -Begin reading core data
        self.thread_param_updater = threading.Timer(0, self.update_parameters_from_topic)
        self.thread_param_updater.daemon = True
        self.thread_param_updater.start()

    def publish_sensors(self):
        try:
            imu_data = self.imu
            comp_data = self.hdg
            self.AUV_IMU.set_data(imu_data)
            self.AUV_COMPASS.set_data(comp_data)
            self.topic_publisher(topic=self.AUV_IMU)
            self.topic_publisher(topic=self.AUV_COMPASS)
        except Exception as e:
            print("publish sensors failed")
            print(e)

    def publish_thrusters(self):
        time.sleep(2)
        while not rospy.is_shutdown():
            try:
                thrusters = self.AUV_GET_THRUSTERS.get_data()
                self.channels = [1500] * 18
                if thrusters != None:
                    self.channels = list(thrusters.channels)
                    print(f"Pre: {self.channels}")
                thruster_data = mavros_msgs.msg.OverrideRCIn()
                if self.DepthHoldMode:
                    self.channels[2] = self.depthMotorPower
                thruster_data.channels = self.channels
                # print(f"Post: {thruster_data.channels}")
                self.TOPIC_SET_RC_OVR.set_data(thruster_data)
                self.topic_publisher(topic=self.TOPIC_SET_RC_OVR)
            except Exception as e:
                print("Thrusters failed")
                print(e)
            time.sleep(0.1)

    def get_sensors(self):
        while not rospy.is_shutdown():
            if self.connected:
                try:
                    self.hdg = self.TOPIC_GET_CMP_HDG.get_data_last()
                    self.imu = self.TOPIC_GET_IMU_DATA.get_data_last()
                    armRequest = self.AUV_GET_ARM.get_data()
                    modeRequest = self.AUV_GET_MODE.get_data()
                    if armRequest != None:
                        self.armRequest = armRequest.data
                        while auv.armed != self.armRequest:
                            self.arm(self.armRequest)
                            time.sleep(2)
                    if modeRequest != None:
                        self.modeRequest = modeRequest.data
                        while self.mode != self.modeRequest:
                            self.change_mode(self.modeRequest)
                            time.sleep(0.5)
                    self.publish_sensors()
                except Exception as e:
                    print("sensor failed")
                    print(e)
                time.sleep(0.1)

    def update_parameters_from_topic(self):
        while not rospy.is_shutdown():
            if self.connected:
                try:
                    data = self.TOPIC_STATE.get_data_last()
                    self.armed = data.armed
                    self.mode = data.mode
                    self.guided = data.guided
                except Exception as e:
                    print("state failed")
                    print(e)
                time.sleep(0.05)

    def beginThreads(self):
        self.thread_sensor_updater = threading.Timer(0, self.get_sensors)
        self.thread_sensor_updater.daemon = True
        self.thread_sensor_updater.start()

        self.thread_override = threading.Timer(0, self.publish_thrusters)
        self.thread_override.daemon = True
        self.thread_override.start()

        while not rospy.is_shutdown():
            pass


def main():
    while not auv.connected:
        print("Waiting to connect...")
        time.sleep(0.5)
    print("Connected!")
    auv.change_mode(MODE_ALTHOLD)
    auv.calibrateDepth()
    time.sleep(2)
    while not auv.armed:
        print("Attempting to arm...")
        auv.arm(True)
        time.sleep(3)
    print("Armed!")
    print("\nNow beginning loops...")
    auv.beginThreads()


def onExit(signum, frame):
    try:
        print("\nDisarming and exiting...")
        auv.arm(False)
        time.sleep(3)
        rospy.signal_shutdown("Rospy Exited")
        while not rospy.is_shutdown():
            pass
        print("\n\nCleanly Exited")
        exit(1)
    except:
        pass


signal.signal(signal.SIGINT, onExit)

if __name__ == "__main__":
    auv = AUV()
    auv.enable_topics_for_read()
    thread_sensor_updater = threading.Timer(0, main)
    thread_sensor_updater.daemon = True
    thread_sensor_updater.start()
    auv.connect("pixStandalone", rate=20)  # change rate to 10 if issues arrive
