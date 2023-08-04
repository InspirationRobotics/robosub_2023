import time

from auv.mission import cointoss_mission, buoy_mission, gate_mission, style_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems.modems_api import Modem, on_receive_msg_logging
import rospy
import os


dock_heading = 100 #chnage
gate_heading = 62 #change

rospy.init_node("missions", anonymous=True)
time.sleep(30)

# load sub config
config = deviceHelper.variables
rc = robot_control.RobotControl()

fail_modem = False

try:
    modem = Modem(on_receive_msg=on_receive_msg_logging)
except:
    fail_modem = True
    print("Failed to start modem, sleeping 15 seconds")

arm.arm()

if not fail_modem:
    modem.send_msg("start onyx")
    modem.send_msg("coin toss")

# Run coin toss
coin_toss = cointoss_mission.CoinTossMission()
time.sleep(2)
coin_toss.run(dock_heading)
coin_toss.cleanup()

rc.forwardDist(5, 2)
time.sleep(1)
rc.setHeadingOld(gate_heading)

if not fail_modem:
    modem.send_msg("coin toss end")
    modem.send_msg("gate")

target = "abydos"
gateMission = gate_mission.GateMission(target)
gateMission.run()
gateMission.cleanup()

if not fail_modem:
    modem.send_msg("gate end")
    modem.send_msg("style")

styleMission = style_mission.StyleMission()
styleMission.run(gate_heading)
styleMission.cleanup()


if not fail_modem:
    modem.send_msg("style end")
    modem.send_msg("buoy")

# Run dhd approach
rc.forwardDist(1.5, 2)
rc.set_depth(1)
time.sleep(4)
buoyMission = buoy_mission.BuoyMission(target)
buoyMission.run()
buoyMission.cleanup()

if not fail_modem:
    modem.send_msg("buoy end")

time.sleep(2)
rc.set_depth(1)
time.sleep(4)
rc.set_depth(0.5)

disarm.disarm() # just in case