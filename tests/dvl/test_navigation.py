import math

import pytest

from auv.device.dvl.dummy_rc import DummyRobotControl


def test_rc_movement():
    rc = DummyRobotControl(pos=[0, 0, 0], heading=0, viz=True)

    # move forward for 1 second
    rc.movement(lateral=0, forward=1, yaw=0, dt=1)


def test_dvl_nav_rel():
    rc = DummyRobotControl(pos=[0, 0, 0], heading=0, viz=True)
    rc.navigate_dvl(x=5, y=-5, z=0, end_heading=30, update_freq=10)


def test_dvl_nav_abs():
    rc = DummyRobotControl(pos=[0, 0, 0], heading=0, viz=True)
    rc.navigate_dvl(x=-5, y=5, z=0, end_heading=-30, relative_coord=False, relative_heading=False, update_freq=10)


def test_dvl_nav_triangle():
    rc = DummyRobotControl(pos=[0, 0, 0], heading=90, viz=True)
    rc.navigate_dvl(x=5, y=-5, z=0, relative_coord=False, relative_heading=False, update_freq=10)
    rc.navigate_dvl(x=0, y=-7, z=0, relative_coord=False, relative_heading=False, update_freq=10)
    rc.navigate_dvl(x=0, y=0, z=0, relative_coord=False, relative_heading=False, update_freq=10)
