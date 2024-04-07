"""
Runs the cointoss mission, gate mission, and then the style mission
"""

from auv.mission import gate_mission, cointoss_mission, style_mission
from auv.utils import arm, disarm, deviceHelper

import rospy
rospy.init_node("initial mission", anonymous=True)

# Get the configuration of the sub's devices, and arm the sub (set mode to autonomous)
config = deviceHelper.variables
arm.arm()

# Run the Cointoss mission
cointossMission = cointoss_mission.CoinTossMission()
cointossMission.run()
cointossMission.cleanup()

# Set the target side of the gate to be the Abydos side
target = "abydos"

# Run the Gate mission
gateMission = gate_mission.GateMission(target)
gateMission.run()
gateMission.cleanup()

# Run the Style mission
styleMission = style_mission.StyleMission()
styleMission.run()
styleMission.cleanup()

