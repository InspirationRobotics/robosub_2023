from auv.mission import template_mission

import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


# create the mission object
mission = template_mission.TemplateMission()

# run the mission
mission.run()

# terminate the mission
mission.cleanup()

# end
logger.info("Mission ended")