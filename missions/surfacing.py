import os
from auv.mission import gate_mission, surfacing_mission

from dotenv import dotenv_values
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

config = dotenv_values(os.environ.get("ENV_PATH", "../config/onyx.env"))


# create the mission object
surfacingMission = surfacing_mission.SurfacingMission(**config)

# run the mission
surfacingMission.run()

# terminate the mission
surfacingMission.cleanup()

# end
logger.info("Mission ended")
