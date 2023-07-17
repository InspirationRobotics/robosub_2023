<<<<<<< HEAD
from auv.mission import gate_mission
=======
import os
from auv.mission import gate_mission, surfacing_mission
>>>>>>> 9eb0f3a0a93f1af2e475748018219ad7adacb8f3

from dotenv import dotenv_values
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

config = dotenv_values(os.environ.get("ENV_PATH", "../config/onyx.env"))

# create the mission object
gateMission = gate_mission.GateMission(**config)

# run the mission
gateMission.run()

# terminate the mission
gateMission.cleanup()

# end
logger.info("Mission ended")