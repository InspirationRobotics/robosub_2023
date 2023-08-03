import time

from auv.mission import cointoss_mission, buoy_mission, gate_mission, style_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
import rospy

rospy.init_node("missions", anonymous=True)

time.sleep(30)

# load sub config
config = deviceHelper.variables
rc = robot_control.RobotControl()
arm.arm()

modem = Modem()
handshake_start(modem)
heading = 218

sleep(15)

# Run coin toss
coin_toss = cointoss_mission.CoinTossMission()
time.sleep(2)
coin_toss.run(heading) # NOTE: TWEAK THIS BEFORE MISSION
coin_toss.cleanup()

rc.forwardDist(6, 2)

target = "abydos"
gateMission = gate_mission.GateMission(target)
gateMission.run()
gateMission.cleanup()

styleMission = style_mission.StyleMission()
styleMission.run(heading)
styleMission.cleanup()

# Run dhd approach
buoyMission = buoy_mission.BuoyMission(target)
buoyMission.run()
buoyMission.cleanup()

disarm.disarm() # just in case