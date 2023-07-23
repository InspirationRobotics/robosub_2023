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

from ..utils import statusLed, deviceHelper
from ..utils.rospyHandler import RosHandler
from ..utils.topicService import TopicService

MODE_MANUAL = "MANUAL"
MODE_STABILIZE = "STABILIZE"
MODE_ALTHOLD = "ALT_HOLD"
MODE_LOITER = "LOITER"
MODE_AUTO = "AUTO"
MODE_GUIDED = "GUIDED"

config = deviceHelper.variables


class AUV(RosHandler):
    def __init__(self):
        self.config = config

        super().__init__()
        self.armed = False
        self.guided = False
        self.mode = ""

        # depth hold
        self.depth = None
        self.do_hold_depth = False
        self.depth_pwm = 0
        self.depth_calib = 0
        self.depth_pid_params = config.get("depth_pid_params", [200, 0.5, 0.1])
        self.depth_pid_offset = config.get("depth_pid_offset", 1500)
        self.depth_pid = PID(*self.depth_pid_params, setpoint=0.5)
        self.depth_pid.output_limits = (-self.depth_pid_params[0], self.depth_pid_params[0])

        # init topics
        self.TOPIC_STATE = TopicService("/mavros/state", mavros_msgs.msg.State)
        self.SERVICE_ARM = TopicService("/mavros/cmd/arming", mavros_msgs.srv.CommandBool)
        self.SERVICE_SET_MODE = TopicService("/mavros/set_mode", mavros_msgs.srv.SetMode)
        self.SERVICE_SET_PARAM = TopicService("/mavros/param/set", mavros_msgs.srv.ParamSet)
        self.SERVICE_GET_PARAM = TopicService("/mavros/param/get", mavros_msgs.srv.ParamGet)

        # movement
        # only works in auto/guided mode
        self.TOPIC_SET_VELOCITY = TopicService("/mavros/setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist)
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
        self.AUV_GET_REL_DEPTH = TopicService("/auv/devices/rel_depth", std_msgs.msg.Float64)
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
            self.do_hold_depth = True
            self.change_mode(MODE_STABILIZE, True)
            return
        self.do_hold_depth = flag
        data = mavros_msgs.srv.SetModeRequest()
        data.custom_mode = mode
        self.SERVICE_SET_MODE.set_data(data)
        result = self.service_caller(self.SERVICE_SET_MODE, timeout=30)
        return result.mode_sent

    def calibrate_depth(self, sample_time=3):
        print("\n[depth_calib] Starting Depth Calibration...")
        samples = []

        # wait for depth data
        while self.depth == None:
            pass
        
        prevDepth = self.depth
        start_time = time.time()

        # get depth data for sample_time seconds
        # then take the mean of the data
        while time.time() - start_time < sample_time:
            if self.depth == prevDepth:
                continue

            samples.append(self.depth)
            prevDepth = self.depth

        self.depth_calib = mean(samples)
        print(f"[depth_calib] Finished. Surface is: {self.depth_calib}")

    def depth_hold(self, depth):
        try:
            depth = depth - self.depth_calib
            if depth < -9 or depth > 100:
                return
            self.depth_pwm = int(self.depth_pid(depth) * -1 + self.depth_pid_offset)
            print(f"[depth_hold] depth: {depth} depthMotorPower: {self.depth_pwm} Target: {self.depth_pid.setpoint}")
            print(f"Depth: {depth:.4f} depthMotorPower: {self.depth_pwm} Target: {self.depth_pid.setpoint}")
            # assume motor range is 1200-1800 so +-300
        except Exception as e:
            print("DepthHold error")
            print(e)

    def get_baro(self, baro):
        try:
            if baro.msgid == 143:
                p = pack("QQ", *baro.payload64)
                time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p)  # pressure is in mBar

                # calculate depth
                press_diff = round(press_diff, 2)
                press_abs = round(press_abs, 2)
                self.depth = (press_abs / (997.0474 * 9.80665 * 0.01)) - self.depth_calib

                # publish baro data
                baro_data = std_msgs.msg.Float32MultiArray()
                baro_data.data = [self.depth, press_diff]
                self.AUV_BARO.set_data(baro_data)
                self.topic_publisher(topic=self.AUV_BARO)
                
                # hold depth
                if self.do_hold_depth and self.armed:
                    self.depth_hold(self.depth)
        except Exception as e:
            print("Baro Failed")
            print(e)

    def set_depth(self, depth):
        if depth.data < 0:
            return
        self.depth_pid.setpoint = depth.data

    def set_rel_depth(self, relative_depth):
        new_depth = self.depth_pid.setpoint + relative_depth.data
        if new_depth < 0:
            return
        self.depth_pid.setpoint = new_depth

    def batteryIndicator(self, msg):
        if self.config.get("battery_indicator", False):
            self.voltage = msg.voltage
            if self.voltage < 13.5:
                statusLed.flashRed()

    def enable_topics_for_read(self):
        self.topic_subscriber(self.TOPIC_STATE)
        self.topic_subscriber(self.TOPIC_GET_IMU_DATA)
        self.topic_subscriber(self.TOPIC_GET_CMP_HDG)
        self.topic_subscriber(self.TOPIC_GET_RC)
        self.topic_subscriber(self.AUV_GET_THRUSTERS)
        self.topic_subscriber(self.AUV_GET_ARM)
        self.topic_subscriber(self.AUV_GET_MODE)
        self.topic_subscriber(self.TOPIC_GET_MAVBARO, self.get_baro)
        self.topic_subscriber(self.AUV_GET_DEPTH, self.set_depth)
        self.topic_subscriber(self.AUV_GET_REL_DEPTH, self.set_rel_depth)
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
                if self.do_hold_depth:
                    self.channels[2] = self.depth_pwm

                thruster_data = mavros_msgs.msg.OverrideRCIn()
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
    auv.calibrate_depth()
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
        rospy.signal_shutdown("Rospy Exited")
        time.sleep(1)
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
