import logging
import os
import time

import yaml
from dotenv import dotenv_values

from auv.mission import gate_mission
from auv.utils import arm

# load sub config and logging config (usefull to see the logs in the console)
config = dotenv_values(os.environ.get("ENV_PATH", "../config/onyx.env"))
log_config = yaml.safe_load(open(os.environ.get("LOG_CFG_PATH", "../config/logging.yaml"), "r"))

logging = logging.config.dictConfig(log_config)
logger = logging.getLogger(__name__)

# arm.arm()
# time.sleep(5)

# create the mission object
gateMission = gate_mission.GateMission(**config)

# run the mission
gateMission.run()

# terminate the mission
gateMission.cleanup()

# end
logger.info("Mission ended")
