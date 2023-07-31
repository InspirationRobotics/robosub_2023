import math

import pytest

from auv.device.dvl import DVL


def test_dvl_invalid_packet():
    dvl = DVL(autostart=False, test=True)

    # set velocity to foward and 0 heading
    dvl.compass_rad = 0
    packet = {
        "vx": 0,
        "vy": 1,
        "vz": 0,
        "valid": False,
    }

    # process packet
    ret = dvl.process_packet(packet)
    assert ret is False and dvl.is_valid is False

def test_dvl_invalid_compass():
    dvl = DVL(autostart=False, test=True)

    # set compass to None
    dvl.compass_rad = None
    packet = {
        "vx": 0,
        "vy": 1,
        "vz": 0,
        "valid": True,
    }

    # process packet
    ret = dvl.process_packet(packet)
    assert ret is False and dvl.is_valid is False

def test_dvl_valid_packet():
    dvl = DVL(autostart=False, test=True)

    dvl.compass_rad = 0
    packet = {
        "vx": 0,
        "vy": 0,
        "vz": 0,
        "valid": True,
        "time": 0,
    }

    # process packet
    ret = dvl.process_packet(packet)
    # (on first iter ret is False because we cannot calculate dt)
    assert ret is False and dvl.is_valid is False

    packet = {
        "vx": 0.01,
        "vy": 0.5,
        "vz": 0,
        "valid": True,
        "time": 1,
    }
    ret = dvl.process_packet(packet)
    assert ret is True and dvl.is_valid is True
    assert dvl.position == pytest.approx([0.01, 0.5, 0], abs=0.001)
    assert dvl.vel_rot == pytest.approx([0.01, 0.5, 0], abs=0.001)

def test_dvl_heading():
    dvl = DVL(autostart=False, test=True)

    dvl.compass_rad = 0
    packet = {
        "vx": 0,
        "vy": 0,
        "vz": 0,
        "valid": True,
        "time": 0,
    }

    # process packet
    ret = dvl.process_packet(packet)
    # (on first iter ret is False because we cannot calculate dt)
    assert ret is False and dvl.is_valid is False

    # heading NORTH
    dvl.compass_rad = math.radians(0)
    packet = {
        "vx": 0.01,
        "vy": 0.5,
        "vz": 0,
        "valid": True,
        "time": 1,
    }
    ret = dvl.process_packet(packet)
    assert ret is True and dvl.is_valid is True
    assert dvl.position == pytest.approx([0.01, 0.5, 0], abs=0.001)
    assert dvl.vel_rot == pytest.approx([0.01, 0.5, 0], abs=0.001)

    # heading EAST
    dvl.compass_rad = math.radians(90)
    packet = {
        "vx": 0.01,
        "vy": 0.5,
        "vz": 0,
        "valid": True,
        "time": 2,
    }
    ret = dvl.process_packet(packet)
    assert ret is True and dvl.is_valid is True
    assert dvl.position == pytest.approx([0.51, 0.49, 0], abs=0.001)
    assert dvl.vel_rot == pytest.approx([0.50, -0.01, 0], abs=0.001)

    # heading SOUTH
    dvl.compass_rad = math.radians(180)
    packet = {
        "vx": 0.01,
        "vy": 0.5,
        "vz": 0,
        "valid": True,
        "time": 3,
    }
    ret = dvl.process_packet(packet)
    assert ret is True and dvl.is_valid is True
    assert dvl.position == pytest.approx([0.50, -0.01, 0], abs=0.001)
    assert dvl.vel_rot == pytest.approx([-0.01, -0.50, 0], abs=0.001)

    # heading WEST
    dvl.compass_rad = math.radians(270)
    packet = {
        "vx": 0.01,
        "vy": 0.5,
        "vz": 0,
        "valid": True,
        "time": 4,
    }
    ret = dvl.process_packet(packet)
    assert ret is True and dvl.is_valid is True
    # we just rotated 360 degrees, so we should be back at the origin
    assert dvl.position == pytest.approx([0, 0, 0], abs=0.001)
    assert dvl.vel_rot == pytest.approx([-0.50, 0.01, 0], abs=0.001)

def test_dvl_context_manager():
    dvl = DVL(autostart=False, test=True)

    dvl.compass_rad = 42
    dvl.position = [1, 2, 3]
    dvl.error = [0.5, 0.5, 0]

    with dvl:
        assert dvl.compass_rad == 42
        assert dvl.position == [0, 0, 0]
        assert dvl.error == [0, 0, 0]
        assert dvl.position_memory[-1] == [1, 2, 3]
        assert dvl.error_memory[-1] == [0.5, 0.5, 0]

    assert dvl.compass_rad == 42
    assert dvl.position == [1, 2, 3]
    assert dvl.error == [0.5, 0.5, 0]

    with dvl:
        dvl.position = [4, 5, 6]
        dvl.error = [0.1, 0.2, 0.3]

    assert dvl.position == [5, 7, 9]
    assert dvl.error == [0.6, 0.7, 0.3]