from auv.mission import surfacing_mission

import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


# create the mission object
surfacingMission = surfacing_mission.SurfacingMission()

# run the mission
surfacingMission.run()

# terminate the mission
surfacingMission.cleanup()

# end
logger.info("Mission ended")