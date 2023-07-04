import os
import json

class Record():
    """Class to record sensor data and store it in a file"""

    def __init__(self, filename, mode="a"):
        self._filename = filename
        self._mode = mode
        self._file = open(filename, mode)

    def write(self, timestamp, angle, data):
        """Append data to the file"""
        self._file.write(json.dumps({"timestamp": timestamp, "angle": angle, "data": data}) + "\n")

    def __del__(self):
        self._file.close()


class Playback():
    """Class to read sensor data from a file and play it back"""

    def __init__(self, filename):
        self._filename = filename
        self._file = open(filename, "r")

    def read(self):
        """Read data from the file and yield it"""
        for line in self._file:
            data = json.loads(line)
            yield data["timestamp"], data["angle"], data["data"]

    def __del__(self):
        self._file.close()