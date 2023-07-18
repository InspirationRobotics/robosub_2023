import os
import time

from auv.mission import surfacing_mission
from auv.utils import arm

from dotenv import dotenv_values
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

config = dotenv_values(os.environ.get("ENV_PATH", "../config/onyx.env"))

arm.arm()
time.sleep(5)


# create the mission object
surfacingMission = surfacing_mission.SurfacingMission(**config)

# run the mission
surfacingMission.run()

# terminate the mission
surfacingMission.cleanup()

# end
logger.info("Mission ended")
