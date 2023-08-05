import time

from auv.mission import gate_mission, cointoss_mission, surfacing_mission, dhd_approach_mission, gate2_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems.modems_api import Modem, on_receive_msg_logging
import rospy


rospy.init_node("missions", anonymous=True)
# time.sleep(30)

# load sub config
config = deviceHelper.variables
rc = robot_control.RobotControl()

fail_modem = False

try:
    modem = Modem(on_receive_msg=on_receive_msg_logging)
    modem.send_msg("graey handshake")
except:
    fail_modem = True
    print("Failed to start modem, starting directly")

arm.arm()

rc.set_depth(0.75)

if not fail_modem:
    modem.send_msg("start graey")
    modem.send_msg("set heading")

time.sleep(2)
rc.forwardDist(3, 2)
target = "earth"
gateMission = gate_mission.GateMission(target)
gateMission.run()
gateMission.cleanup()

if not fail_modem:
    modem.send_msg("set heading end")
    modem.send_msg("traveling")

t = 0
while t < 40:  # 50 seconds in transdec
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
