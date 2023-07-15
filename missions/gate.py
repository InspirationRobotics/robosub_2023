from auv.mission import template

import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


# create the mission object
gate_mission = template.TemplateMission()

# run the mission
gate_mission.run()

# terminate the mission
gate_mission.terminate()

# end
logger.info("Mission ended")