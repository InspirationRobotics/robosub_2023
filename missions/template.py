from auv.mission import template

import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


# create the mission object
mission = template.TemplateMission()

# run the mission
mission.run()

# terminate the mission
mission.terminate()

# end
logger.info("Mission ended")