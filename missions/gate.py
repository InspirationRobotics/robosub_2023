from auv.mission import gate_mission, surfacing_mission

import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


# create the mission object
gateMission = gate_mission.GateMission()

# run the mission
gateMission.run()

# terminate the mission
gateMission.cleanup()

# end
logger.info("Mission ended")