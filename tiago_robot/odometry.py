"""Odometry module — GPS + compass fusion for robot localization."""

import numpy as np


class Odometry:
    """Updates robot position (xw, yw) and orientation (alpha) from sensors."""

    def update_odometry(self):
        self.xw = self.gps.getValues()[0]
        self.yw = self.gps.getValues()[1]
        self.alpha = np.arctan2(
            self.compass.getValues()[0], self.compass.getValues()[1]
        )
