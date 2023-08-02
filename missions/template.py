import os
import time

from auv.mission import template_mission
from auv.utils import arm, deviceHelper

import rospy
rospy.init_node("missions", anonymous=True)

# load sub config
config = deviceHelper.variables

# arm.arm()
# time.sleep(5)

# create the mission object
mission = template_mission.TemplateMission(**config)

# run the mission
mission.run()

# terminate the mission
mission.cleanup()

# end
print("[INFO] Mission ended")
