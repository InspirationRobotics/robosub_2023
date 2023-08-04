import time

from auv.mission import cointoss_mission, surfacing_mission, dhd_approach_mission, gate2_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems.modems_api import Modem, handshake_start
import rospy

rospy.init_node("missions", anonymous=True)
time.sleep(30)

# load sub config
config = deviceHelper.variables
rc = robot_control.RobotControl()
arm.arm()

# modem = Modem()
# handshake_start(modem)
gate_heading = 223
octagon_heading = 240
rc.forwardDist(5, 2)
# Run coin toss
coin_toss = cointoss_mission.CoinTossMission(**config)
time.sleep(2)
coin_toss.run(gate_heading)  # NOTE: TWEAK THIS BEFORE MISSION
coin_toss.cleanup()

gate = gate2_mission.Gate2Mission()
gate.run()
gate.cleanup()
coin_toss.run(octagon_heading)
coin_toss.cleanup()
t = 0
while t < 50: #50 seconds in transdec
    rc.movement(forward=2)
    time.sleep(0.1)
    t += 0.1

t = 0
while t < 1:
    rc.movement(forward=0)
    time.sleep(0.1)
    t += 0.1

# Run dhd approach
dhd_app = dhd_approach_mission.DHDApproachMission(**config)
time.sleep(2)
dhd_app.run()
dhd_app.cleanup()

# Run surfacing
surfacing = surfacing_mission.SurfacingMission(**config)
time.sleep(2)
surfacing.run()
surfacing.cleanup()

disarm.disarm()  # just in case
