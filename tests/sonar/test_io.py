import os

import pytest

from auv.device.sonar.io import Record, Playback


def test_record():
    filename = "test.txt"
    r = Record(filename, mode="w")

    r.write(1, 5, [255, 127, 1])
    r.write(2, 10, [1, 2, 3])
    del r  # close the file

    assert os.path.exists(filename)
    assert os.path.getsize(filename) > 0

    with open(filename, "r") as f:
        lines = f.readlines()

    assert len(lines) == 2
    assert lines[0] == '{"timestamp": 1, "angle": 5, "data": [255, 127, 1]}\n'
    assert lines[1] == '{"timestamp": 2, "angle": 10, "data": [1, 2, 3]}\n'


def test_record_append():
    filename = "test.txt"
    r = Record(filename, mode="a")

    r.write(3, 15, [4, 5, 6])
    del r  # close the file

    assert os.path.exists(filename)
    assert os.path.getsize(filename) > 0

    with open(filename, "r") as f:
        lines = f.readlines()

    assert len(lines) == 3
    assert lines[2] == '{"timestamp": 3, "angle": 15, "data": [4, 5, 6]}\n'


def test_playback():
    filename = "test.txt"
    pb = Playback(filename)

    data = list(pb)

    assert len(data) == 3
    assert data[0] == (1, 5, [255, 127, 1])
    assert data[1] == (2, 10, [1, 2, 3])
    assert data[2] == (3, 15, [4, 5, 6])


def test_cleanup():
    if os.path.exists("test.txt"):
        os.remove("test.txt")

    assert not os.path.exists("test.txt")
