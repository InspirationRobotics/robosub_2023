from auv.mission import path_mission

import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


# create the mission object
pathMission = path_mission.PathMission()

# run the mission
pathMission.run()

# terminate the mission
pathMission.cleanup()

# end
logger.info("Mission ended")