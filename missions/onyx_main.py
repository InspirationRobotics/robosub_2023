import time

from auv.mission import cointoss_mission, buoy_mission, gate_mission, style_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems.modems_api import Modem, handshake_start
import rospy
import os

rospy.init_node("missions", anonymous=True)

time.sleep(60)

# load sub config
config = deviceHelper.variables
rc = robot_control.RobotControl()

fail_modem = False

try:
    modem = Modem()
    handshake_start(modem)
except:
    fail_modem = True
    time.sleep(30)
    print("Failed to start modem, sleeping 15 seconds")

time.sleep(20)
arm.arm()
heading = 62

if not fail_modem:
    modem.send_msg("start onyx")


if not fail_modem:
    modem.send_msg("coin toss")

# Run coin toss
coin_toss = cointoss_mission.CoinTossMission()
time.sleep(2)
coin_toss.run(heading) # NOTE: TWEAK THIS BEFORE MISSION
coin_toss.cleanup()

if not fail_modem:
    modem.send_msg("coin toss end")

rc.forwardDist(6, 2)

if not fail_modem:
    modem.send_msg("gate")

target = "abydos"
gateMission = gate_mission.GateMission(target)
gateMission.run()
gateMission.cleanup()

if not fail_modem:
    modem.send_msg("gate end")

if not fail_modem:
    modem.send_msg("style")

styleMission = style_mission.StyleMission()
styleMission.run(heading)
styleMission.cleanup()


if not fail_modem:
    modem.send_msg("style end")


if not fail_modem:
    modem.send_msg("buoy")

# Run dhd approach
buoyMission = buoy_mission.BuoyMission(target)
buoyMission.run()
buoyMission.cleanup()

if not fail_modem:
    modem.send_msg("buoy end")

disarm.disarm() # just in case