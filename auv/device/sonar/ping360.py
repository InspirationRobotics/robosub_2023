import time

import brping
import numpy as np

from . import utils


class Ping360(brping.Ping360):
    speed_of_sound = 1500.0  # meters per second
    sample_period_tick_duration = 25e-9  # seconds

    min_transmit_duration = 5
    max_transmit_duration = 200
    max_duration_ratio = 64e6

    max_samples = 1000
    min_sample_period = 400

    def __init__(
        self,
        device,
        baudrate=115200,
        scan_mode=0,
        angle_range=(0, 399),
        angle_step=1,
        max_range=10,
        gain=2,
        transmit_freq=800,
    ):
        """Ping 360 sonar class

        Args:
            device (str): serial device
            baudrate (int, optional): Defaults to 115200.
            scan_mode (int, optional): 0 = full scan, 1 = sector scan. Defaults to 0.
            angle_range (tuple, optional): angle range for the scan, center is 200. Defaults to (0, 399).
            angle_step (int, optional): how many degrees to step between each scan. Defaults to 1.
            max_range (int, optional): distance to scan. Defaults to 10.
            gain (int, optional): gain setting. Defaults to 2.
            transmit_freq (int, optional): transmit frequency. Defaults to 800.

        Raises:
            RuntimeError: raised if the Ping360 fails to initialize
        """
        super().__init__()
        self.connect_serial(device, baudrate)

        if not self.initialize():
            raise RuntimeError("failed to initialize Ping360")

        self._scan_mode = self.set_scan_mode(scan_mode)
        self._angle_range = self.set_angle_range(angle_range)
        self._angle_step = self.set_angle_step(angle_step)
        self._angle = self._angle_range[0]
        self._increment = self._angle_step

        self.set_transmit_frequency(transmit_freq)
        self.set_max_range(max_range)
        self.set_gain_setting(gain)

        print("[DEBUG] Ping360 initialized")

    def set_scan_mode(self, scan_mode):
        if scan_mode != 0 and scan_mode != 1:
            raise ValueError("scan_mode must be 0 or 1")

        self._scan_mode = scan_mode
        return self._scan_mode

    def set_angle_range(self, angle_range):
        if angle_range[0] < 0 or angle_range[1] > 399 or angle_range[0] > angle_range[1]:
            raise ValueError(f"invalid angle range: {angle_range}")

        self._angle_range = (angle_range[0] % 400, angle_range[1] % 400)
        self._angle = self._angle_range[0]
        return self._angle_range

    def set_angle_step(self, angle_step):
        if angle_step < 1 or angle_step > 20:
            raise ValueError(f"invalid angle step: {angle_step}")

        self._angle_step = angle_step
        self._increment = angle_step
        return self._angle_step

    def set_max_range(self, max_range):
        """Set the max range for the scan

        Args:
            max_range (int): distance to scan in meters

        Returns:
            int: distance set
        """
        if max_range < 1 or max_range > 50:
            raise ValueError(f"invalid max range: {max_range}")

        self._max_range = max_range
        self._number_of_samples = int(
            min(
                self.max_samples,
                2 * self._max_range / (self.sample_period_tick_duration * self.min_sample_period * self.speed_of_sound),
            )
        )
        self._sample_period = int(
            2 * self._max_range / (self._number_of_samples * self.sample_period_tick_duration * self.speed_of_sound)
        )

        self._transmit_duration = int(
            max(
                self._sample_period * self.sample_period_tick_duration / 400,
                (8000 * self._max_range) / self.speed_of_sound,
            )
        )

        self.norm_dist_factor = self._max_range / self._number_of_samples

    def __next__(self):
        """Get a step scan from the sensor and return the current angle and distances"""

        # update angle
        if self._scan_mode == 0:
            # loop when angle reaches end of range
            if self._angle >= self._angle_range[1]:
                self._angle = self._angle_range[0]

            self._angle += self._angle_step

        elif self._scan_mode == 1:
            # reverse direction when angle reaches end of range
            if self._angle <= self._angle_range[0] and self._increment < 0:
                self._increment = self._angle_step
            elif self._angle >= self._angle_range[1] and self._increment > 0:
                self._increment = -self._angle_step

            self._angle += self._increment

        # read sensor
        self.transmitAngle(self._angle)
        return time.time(), self._angle, list(self._data)

    def __iter__(self):
        """Stream full scans from the sensor, yielding the data as it receives it."""

        if self._scan_mode == 0:
            # reset angle to start of range
            self._angle = self._angle_range[0]

            while self._angle < self._angle_range[1]:
                yield self.__next__()

        elif self._scan_mode == 1:
            if self._increment > 0:
                self._angle = self._angle_range[0]
            else:
                self._angle = self._angle_range[1]

            curr_increment = self._increment
            while curr_increment == self._increment:
                yield self.__next__()

    def get_polar_image(self):
        """Get a polar image from the sensor

        Returns:
            numpy.ndarray: polar image
        """
        size = (400, self._number_of_samples)
        img = np.zeros((size[0], size[1], 1), dtype=np.uint8)

        # do a full scan for the given range
        for ts, angle, points in self:
            img = utils.plot_to_polar_gray(img, angle, points, imsize=size, step_angle=self._angle_step)

        return img

    def get_obstacles(self, threshold=60, smallest_area=20):
        """Get a list of obstacles by doing a sweep of the sonar

        Returns:
            list: list of obstacles
        """
        img = self.get_polar_image()

        # object detection
        obs = utils.object_detection(
            img,
            dist_factor=self.norm_dist_factor,
            threshold=threshold,
        )

        # filter out small obstacles
        obs = [o for o in obs if o.area > smallest_area]
