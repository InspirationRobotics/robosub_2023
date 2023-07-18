from auv.mission import path_mission
from auv.utils import arm
import time
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

arm.arm()
time.sleep(5)
# create the mission object
pathMission = path_mission.PathMission()

# run the mission
pathMission.run()

# terminate the mission
pathMission.cleanup()

# end
logger.info("Mission ended")