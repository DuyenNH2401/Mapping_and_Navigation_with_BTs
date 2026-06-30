"""Odometry module — GPS + compass fusion for robot localization."""

import numpy as np


class Odometry:
    """Updates robot position (xw, yw) and orientation (alpha) from sensors."""

    def update_odometry(self):
        # Get the robot's position and orientation from GPS and Compass
        d = 0.23
        position = self.gps.getValues()

        compass_values = self.compass.getValues()
        self.alpha = np.arctan2(
            compass_values[0], compass_values[1]
        )  # Calculate the yaw angle from compass readings

        self.xw = position[0] - d * np.cos(self.alpha)
        self.yw = position[1] - d * np.sin(self.alpha)
