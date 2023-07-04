import brping
import logging

logger = logging.getLogger(__name__)

class Ping360(brping.Ping360):

    def __init__(self, device, baudrate=115200, scan_mode=0, angle_range=(0, 399), angle_step=1):
        super().__init__()
        self.connect_serial(device, baudrate)
        
        if not self.initialize():
            raise RuntimeError("failed to initialize Ping360")
        
        self._scan_mode = self.set_scan_mode(scan_mode)
        self._angle_range = self.set_angle_range(angle_range)
        self._angle_step = self.set_angle_step(angle_step)
        self._angle = angle_range[0]
        self._increment = self._angle_step

        logger.debug(f"Ping360 initialized")

    def set_scan_mode(self, scan_mode):
        if scan_mode != 0 and scan_mode != 1:
            raise ValueError("scan_mode must be 0 or 1")

        self._scan_mode = scan_mode
        return self._scan_mode
    
    def set_angle_range(self, angle_range):
        if angle_range[0] < 0 or angle_range[1] > 399 or angle_range[0] > angle_range[1]:
            raise ValueError(f"invalid angle range: {angle_range}")

        self._angle_range = angle_range
        self._angle = angle_range[0]
        return self._angle_range

    def set_angle_step(self, angle_step):
        if angle_step < 1 or angle_step > 20:
            raise ValueError(f"invalid angle step: {angle_step}")

        self._angle_step = angle_step
        self._increment = angle_step
        return self._angle_step

    def step_scan(self):
        """Get a step scan from the sensor and return the current angle and distances"""

        # update angle
        if self._scan_mode == 0:
            # loop when angle reaches end of range
            if self._angle >= self._angle_range[1]:
                self._angle = self._angle_range[0]

            self._angle += self._angle_step

        elif self._scan_mode == 1:
            # reverse direction when angle reaches end of range
            if self._angle <= self._angle_range[0]:
                self._increment = self._angle_step
            elif self._angle >= self._angle_range[1]:
                self._increment = -self._angle_step

            self._angle += self._increment

        # read sensor
        self.transmitAngle(self._angle)

        angle, data = self._angle, bytearray(self._data)
        logger.debug(f"angle: {angle}, distances: {data}")
        return angle, data

    def full_scan(self):
        """Get a full scan from the sensor and return the data as a point list of length 400."""

        # create points list
        points = [bytearray() for _ in range(400)]

        # reset angle to start of range
        self._angle = self._angle_range[0]

        while self._angle < self._angle_range[1]:
            
            angle, data = self.step_scan()
            if angle > 0 and angle < 400:
                points[angle] = data

        return points

    def stream_full_scan(self):
        """Stream full scans from the sensor, yielding the data as it receives it."""

        # reset angle to start of range
        self._angle = self._angle_range[0]

        while self._angle < self._angle_range[1]:
            yield self.step_scan()
