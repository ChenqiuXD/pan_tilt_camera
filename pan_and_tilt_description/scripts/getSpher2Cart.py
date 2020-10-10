#!/usr/bin/env python3
from math import cos, sin

RADIUS = 100
droneArg = [[RADIUS, 0, 1.3, 1.0],
            [RADIUS, 0.8, 1.0, 1.0],
            [RADIUS, 1.7, 1.5, 1.0]]

for drone in droneArg:
    z = drone[0] * cos(drone[2])
    y = drone[0] * sin(drone[2]) * sin(drone[1])
    x = drone[0] * sin(drone[2]) * cos(drone[1])
    print("The drone in spher coordinate [", drone[0], ",", drone[1], ",", drone[2], "] is")
    print("x: ", x, "y: ", y, "z: ", z, "\n")

