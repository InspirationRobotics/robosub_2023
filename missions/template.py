import logging
import os

import yaml
from dotenv import dotenv_values

from auv.mission import template_mission

# load sub config and logging config (usefull to see the logs in the console)
config = dotenv_values(os.environ.get("ENV_PATH", "../config/onyx.env"))
log_config = yaml.safe_load(open(os.environ.get("LOG_CFG_PATH", "../config/logging.yaml"), "r"))

logging = logging.config.dictConfig(log_config)
logger = logging.getLogger(__name__)

# create the mission object
mission = template_mission.TemplateMission()

# run the mission
mission.run()

# terminate the mission
mission.cleanup()

# end
logger.info("Mission ended")
