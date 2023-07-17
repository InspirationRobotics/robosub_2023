import os
from auv.mission import gate_mission, surfacing_mission

from dotenv import dotenv_values
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

config = dotenv_values(os.environ.get("ENV_PATH", "../config/onyx.env"))


# create the mission object
mission = template_mission.TemplateMission()

# run the mission
mission.run()

# terminate the mission
mission.cleanup()

# end
logger.info("Mission ended")
