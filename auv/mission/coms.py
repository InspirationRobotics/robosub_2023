import time

from auv.mission import gate_mission, cointoss_mission, surfacing_mission, dhd_approach_mission, gate2_mission
from auv.utils import arm, disarm, deviceHelper
from auv.motion import robot_control
from auv.device.modems.modems_api import Modem, on_receive_msg_logging
import rospy

rospy.init_node("coms_mission", anonymous=True)
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

time.sleep(90)

disarm.disarm()