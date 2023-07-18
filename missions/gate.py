import os
import time

from auv.utils import arm
from auv.mission import gate_mission

from dotenv import dotenv_values
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

config = dotenv_values(os.environ.get("ENV_PATH", "../config/onyx.env"))

#arm.arm()
#time.sleep(5)

# create the mission object
gateMission = gate_mission.GateMission(**config)

# run the mission
gateMission.run()

# terminate the mission
gateMission.cleanup()

# end
logger.info("Mission ended")