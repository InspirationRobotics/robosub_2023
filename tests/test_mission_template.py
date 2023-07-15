"""
Test mission template
run ONLY on the sub when ROS and camVersatile is running
"""
import time

from auv.mission import template_mission

@pytest.mark.sub
def test_mission_template():
    # creates a mission template object
    mission = template_mission.TemplateMission()
    assert mission.data == {}

    # Run the mission
    mission.run()
    time.sleep(2)

    # Cleanup
    mission.cleanup()
    assert len(mission.data.keys()) == len(mission.cv_files)

@pytest.mark.sub
def test_mission_template_del():
    # creates a mission template object
    mission = template_mission.TemplateMission()

    # Run the mission
    mission.run()

    # This should aswell cleanup
    del mission
