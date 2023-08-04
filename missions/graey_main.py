import time

from auv.mission import cointoss_mission, surfacing_mission, dhd_approach_mission, gate2_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems.modems_api import Modem, handshake_start
import rospy


dock_heading = 100 #chnage
gate_heading = 223 #change
octagon_heading = 240 #change

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
    print("Failed to start modem, starting directly")

arm.arm()

if not fail_modem:
    modem.send_msg("start graey")

# Run coin toss
coin_toss = cointoss_mission.CoinTossMission(**config)
time.sleep(2)
coin_toss.run(dock_heading)
coin_toss.cleanup()


if not fail_modem:
    modem.send_msg("gate")

rc.forwardDist(4, 2)
time.sleep(2)
rc.setHeadingOld(gate_heading)

gate = gate2_mission.Gate2Mission()
gate.run()
gate.cleanup()
coin_toss.run(octagon_heading)
coin_toss.cleanup()

if not fail_modem:
    modem.send_msg("gate end")

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

    
if not fail_modem:
    modem.send_msg("dhd approach")

# Run dhd approach
dhd_app = dhd_approach_mission.DHDApproachMission(**config)
time.sleep(2)
dhd_app.run()
dhd_app.cleanup()

    
if not fail_modem:
    modem.send_msg("dhd approach end")
    modem.send_msg("surfacing")

# Run surfacing
surfacing = surfacing_mission.SurfacingMission(**config)
time.sleep(2)
surfacing.run()
surfacing.cleanup()

if not fail_modem:
    modem.send_msg("surfacing end")

disarm.disarm()  # just in case
