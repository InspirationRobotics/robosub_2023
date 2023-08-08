import time
import signal

from auv.mission import gate_mission, cointoss_mission, surfacing_mission, dhd_approach_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems import modems_api
import rospy

myNode = rospy.init_node("missions", anonymous=True)
octagon_heading = 350
time.sleep(30)

# load sub config
config = deviceHelper.variables
rc = robot_control.RobotControl()

try:
    modem = modems_api.Modem()
    modem.send_msg("graey handshake")
    fail_modem = False
except:
    fail_modem = True
    print("Failed to start modem, starting directly")


def onExit(signum, frame):
    try:
        print("\Closing and exiting...")

        # cleanup modems and LED
        modems_api.led.clean()
        if not fail_modem:
            modem.stop()

        myNode.stop()
        time.sleep(3)
        rospy.signal_shutdown("Rospy Exited")
        while not rospy.is_shutdown():
            pass
        print("\n\nCleanly Exited")
        exit(1)
    except:
        pass

signal.signal(signal.SIGINT, onExit)

arm.arm()
rc.set_depth(0.75)

if not fail_modem:
    modem.send_msg("start graey")
    modem.send_msg("set heading")

time.sleep(2)
rc.forwardDist(3.5, 2)
target = "earth"
gateMission = gate_mission.GateMission(target, **config)
gateMission.run()
gateMission.cleanup()

if not fail_modem:
    modem.send_msg("set heading end")
    modem.send_msg("traveling")

rc.setHeadingOld(octagon_heading)
t = 0
while t < 70:  # need to adjust for finals
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
