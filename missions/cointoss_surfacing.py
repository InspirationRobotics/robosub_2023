"""
Runs the Cointoss, the DHD approach, then the Surfacing mission
"""

import time

from auv.mission import cointoss_mission, surfacing_mission, dhd_approach_mission
from auv.utils import arm, disarm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# Load the configuration of the sub's devices
config = deviceHelper.variables
arm.arm()

# Run the Cointoss mission
coin_toss = cointoss_mission.CoinTossMission()
time.sleep(2)
coin_toss.run(218) # NOTE: TWEAK THIS BEFORE MISSION
coin_toss.cleanup()

# Run the DHD approach mission
dhd_app = dhd_approach_mission.DHDApproachMission()
time.sleep(2)
dhd_app.run()
dhd_app.cleanup()

# Run the surfacing mission
surfacing = surfacing_mission.SurfacingMission()
time.sleep(2)
surfacing.run()
surfacing.cleanup()

disarm.disarm() # Disarm the sub explicitly just in case 