"""Calculate the position of the robot given the velocity and heading"""

import matplotlib.pyplot as plt
import math


def next_pos(x, y, vx, vy, heading, dt):
    """Calculate the next position given the current position and velocity"""
    x += (vx * math.cos(math.radians(heading)) + vy * math.sin(math.radians(heading))) * dt
    y += (vx * math.sin(math.radians(heading)) + vy * math.cos(math.radians(heading))) * dt
    return x, y


# (relative, relative, absolute, absolute)
# (vx, vy, heading, time)
data = [
    (0, 0, 0, 1),  # no movement
    (1, 0, 0, 2),  # move forward 1 m/s
    (0, -1, 0, 3),  # move right 1 m/s
    (1, 0, 0, 4),  # move forward 1 m/s
    (0, 1, 0, 5),  # move left 1 m/s
    (1, 0, 90, 6),  # heading 90 degrees (left heading from starting pos) + 1 m/s forward
    (1, 0, 180, 7),  # heading 180 degrees (opposite heading from starting pos) + 1 m/s forward
    (1, 0, 210, 8),  # heading 210 degrees + 1 m/s forward
]

# only used to plot the points
Xs = []
Ys = []

prev_time = 0
x = 0
y = 0

for vx, vy, heading, time in data:
    dt = time - prev_time
    x, y = next_pos(x, y, vx, vy, heading, dt)
    prev_time = time

    print(x, y)

    Xs.append(x)
    Ys.append(y)

# arrow to show the heading (at the start)
plt.arrow(0, 0, math.cos(math.radians(data[0][2])) * 0.1, math.sin(math.radians(data[0][2])) * 0.1, color="red", width=0.05)

plt.plot(Xs, Ys)
plt.show()
