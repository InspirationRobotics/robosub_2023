from auv.mission import gate_mission, cointoss_mission, style_mission
from auv.utils import arm, disarm, deviceHelper

import rospy
rospy.init_node("initial mission", anonymous=True)
# load sub config
config = deviceHelper.variables
arm.arm()

cointossMission = cointoss_mission.CoinTossMission()
cointossMission.run()
cointossMission.cleanup()

target = "abydos"

gateMission = gate_mission.GateMission(target)
gateMission.run()
gateMission.cleanup()

styleMission = style_mission.StyleMission()
styleMission.run()
styleMission.cleanup()

