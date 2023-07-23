"""Dummy AUV with simple physics to test DVL navigation"""
import math
import time

import cv2
import numpy as np
from simple_pid import PID

from .dvl import DVL
from ...motion.utils import get_heading_from_coords, heading_error, rotate_vector, inv_rotate_vector


class DummyRobotControl:
    def __init__(self, pos=[0, 0, 0], heading=45, viz=True, **config):
        self.config = config

        self.dvl = DVL(autostart=False, test=True)
        self.dvl.compass_rad = math.radians(heading)
        self.dvl.position = pos

        self.target_position = [0, 0, 0]
        self.vel = [0, 0, 0]  # absolute coord velocity x, y, z
        self.yaw_rate = 0  # rad/s
        self.time = 0  # seconds

        self.PIDs = {
            "yaw": PID(
                self.config.get("YAW_PID_P", 7),
                self.config.get("YAW_PID_I", 0.01),
                self.config.get("YAW_PID_D", 0.0),
                setpoint=0,
                output_limits=(-2, 2),
            ),
            "forward": PID(
                self.config.get("FORWARD_PID_P", 1.0),
                self.config.get("FORWARD_PID_I", 0.01),
                self.config.get("FORWARD_PID_D", 0.1),
                setpoint=0,
                output_limits=(-2, 2),
            ),
            "lateral": PID(
                self.config.get("LATERAL_PID_P", 1.0),
                self.config.get("LATERAL_PID_I", 0.01),
                self.config.get("LATERAL_PID_D", 0.0),
                setpoint=0,
                output_limits=(-2, 2),
            ),
        }

        # viz stuff
        self.viz = viz
        self.img = np.zeros((500, 500, 3), np.uint8)
        self.heading_size = 50
        self.heading_color = (0, 0, 255)
        self.pos_color = (0, 255, 0)

    def movement(self, lateral=0, forward=0, yaw=0, dt=0.1):
        """
        Map thruster input to velocity,
        thruster input is in range [-2, 2]
        non-linear mapping
        uses compass to convert local velocity to absolute velocity
        """
        if dt:
            self.time += dt
        else:
            self.time = time.time()

        abs_max_vel = 1.5  # m/s
        abs_max_yaw_rate = 1  # rad/s

        # calculate relative velocity m/s
        if lateral >= 0:
            vel_x = abs_max_vel * (1 - math.exp(-lateral))
        else:
            vel_x = -abs_max_vel * (1 - math.exp(lateral))
        if forward >= 0:
            vel_y = abs_max_vel * (1 - math.exp(-forward))
        else:
            vel_y = -abs_max_vel * (1 - math.exp(forward))
        vel_z = 0

        # update compass heading
        if yaw > 0:
            yaw_rate_rad = abs_max_yaw_rate * (1 - math.exp(-yaw))
        else:
            yaw_rate_rad = -abs_max_yaw_rate * (1 - math.exp(yaw))
        self.dvl.compass_rad += yaw_rate_rad * dt
        self.dvl.compass_rad = self.dvl.compass_rad % (2 * math.pi)

        # print(f"[DEBUG] vel_x={vel_x}, vel_y={vel_y}, yaw_rate={yaw_rate_rad}")

        self.dvl.process_packet(
            {
                "AUV_velocity": [vel_x * 1000, vel_y * 1000, vel_z * 1000],
                "isAUV_velocity_valid": True,
                "Time": self.time,
            }
        )

        self.visualize()

    def set_heading(self, target: int):
        """Yaw to target heading, heading is absolute, blocking function"""

        target = (target) % 360
        print(f"[INFO] Setting heading to {target}")

        while True:
            if self.dvl.compass_rad is None:
                print("[WARN] Compass not ready")
                time.sleep(0.5)
                continue

            error = heading_error(math.degrees(self.dvl.compass_rad), target)
            # normalize error to -1, 1 for the PID controller
            output = self.PIDs["yaw"](-error / 180)

            print(f"[DEBUG] Heading error: {error / 180}, output: {output}")

            if abs(error) <= 1:
                print("[INFO] Heading reached")
                break

            self.movement(lateral=0, forward=0, yaw=output, dt=0.1)
            time.sleep(0.1)

        print(f"[INFO] Finished setting heading to {target}")

    def navigate_dvl(self, x, y, z, end_heading=None, relative_coord=True, relative_heading=True, update_freq=10):
        """
        Navigate to a position using DVL
        first rotate to the target heading
        move there on y axis, adjust lateral using x
        then rotate for the end_heading
        """

        # reset PID integrals
        for pid in self.PIDs.values():
            pid.reset()

        self.target_position = [x, y, z]

        if not relative_coord:
            # convert to relative coordinates
            # heading 0 is north so y "+" axis is going to the north
            x -= self.dvl.position[0]
            y -= self.dvl.position[1]

        if relative_heading:
            # rotate [x, y] according to the current heading
            x, y = rotate_vector(x, y, math.degrees(self.dvl.compass_rad))

        target_heading = get_heading_from_coords(x, y)
        end_heading = end_heading % 360 if end_heading is not None else target_heading

        print(f"[INFO] Navigating to {x}, {y}, {z}, heading {target_heading}")

        # rotate and set depth
        self.set_heading(target_heading)
        self.set_depth(z)

        # navigate to the target position
        with self.dvl:
            while True:
                if self.dvl.is_valid is None:
                    print("[WARN] DVL not ready")
                    self.movement(lateral=0, forward=0, yaw=0, dt=1 / update_freq)
                    time.sleep(1 / update_freq)
                    continue

                # calculate error
                err_x, err_y = inv_rotate_vector(
                    x - self.dvl.position[0],
                    y - self.dvl.position[1],
                    math.degrees(self.dvl.compass_rad),
                )

                # check if we reached the target
                x_err_th = 0.1 + self.dvl.error[0]
                y_err_th = 0.1 + self.dvl.error[1]
                if abs(err_x) <= x_err_th and abs(err_y) <= y_err_th:
                    print("[INFO] Target reached")
                    break

                # calculate PID outputs
                output_x = self.PIDs["lateral"](-err_x)
                output_y = self.PIDs["forward"](-err_y)
                print(f"[DEBUG] err_x={err_x}, err_y={err_y}, output_x={output_x}, output_y={output_y}")
                self.movement(lateral=output_x, forward=output_y, dt=1 / update_freq)
                time.sleep(1 / update_freq)

        # rotate to the end heading
        self.set_heading(end_heading)

    def set_depth(self, depth):
        # we don't really care about the depth
        self.dvl.position[2] = depth

    def visualize(self, name="Dummy AUV"):
        if not self.viz:
            return

        self.img = np.zeros((500, 500, 3), np.uint8)

        # draw target position
        target_x = int(self.target_position[0] * 25 + 250)
        target_y = int(self.target_position[1] * 25 + 250)
        cv2.circle(self.img, (target_x, target_y), 5, (255, 0, 0), -1)

        # draw position circle
        if len(self.dvl.position_memory):
            context_x, context_y, _ = self.dvl.position_memory[-1]
        else:
            context_x, context_y = 0, 0
        pos_x = int((context_x + self.dvl.position[0]) * 25 + 250)
        pos_y = int((context_y + self.dvl.position[1]) * 25 + 250)
        cv2.circle(self.img, (pos_x, pos_y), 5, self.pos_color, -1)

        # draw heading arrow, 0 = down (opencv coordinates)
        heading_x = int(self.heading_size * math.sin(self.dvl.compass_rad))
        heading_y = int(self.heading_size * math.cos(self.dvl.compass_rad))
        cv2.arrowedLine(
            self.img,
            (pos_x, pos_y),
            (pos_x + heading_x, pos_y + heading_y),
            self.heading_color,
            2,
        )

        cv2.imshow(name, self.img)
        cv2.waitKey(1)
