import time
import signal

from auv.mission import cointoss_mission, buoy_mission, gate_mission, style_mission, dhd_approach_mission, surfacing_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems import modems_api
import rospy

target = "abydos"
#dock_heading = 100 #chnage
gate_heading = 60 #change
dhdDir = -1 #change (1 is plus 20 which is clockwise)

myNode = rospy.init_node("missions", anonymous=True)
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
        print("Exiting...")
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

if not fail_modem:
    modem.send_msg("start onyx")
    modem.send_msg("coin toss")

# Run coin toss
coin_toss = cointoss_mission.CoinTossMission()
time.sleep(2)
coin_toss.run(gate_heading)
coin_toss.cleanup()

time.sleep(2)
rc.forwardDist(6, 2)
time.sleep(2)
rc.lateralUni(-2,4)
time.sleep(1)
rc.setHeadingOld(gate_heading)

if not fail_modem:
    modem.send_msg("coin toss end")
    modem.send_msg("gate")

gateMission = gate_mission.GateMission(target, **config)
gateMission.run()
gateMission.cleanup()
rc.forwardDist(2, 2)

if not fail_modem:
    modem.send_msg("gate end")
    modem.send_msg("style")

styleMission = style_mission.StyleMission()
styleMission.run(gate_heading-8)
styleMission.cleanup()


if not fail_modem:
    modem.send_msg("style end")
    modem.send_msg("buoy")

# Run buoy approach
rc.forwardDist(5, 2)
rc.set_depth(1)
#time.sleep(4)
buoyMission = buoy_mission.BuoyMission(target)
buoyMission.run()
buoyMission.cleanup()

if not fail_modem:
    modem.send_msg("buoy end")
    modem.send_msg("heading to octagon")

# goes up above buoy and then goes forward and lines up with octagon
time.sleep(2)
rc.set_depth(1)
time.sleep(4)
rc.set_depth(1.3) #0.5
time.sleep(2)
rc.forwardDist(3, 2)
time.sleep(2)
rc.setHeadingOld(gate_heading+(dhdDir*45)) # guesstimating
time.sleep(2)

rc.forwardUni(2, 47)

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

disarm.disarm()