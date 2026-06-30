"""Perception module — LiDAR processing: polar-to-Cartesian conversion."""

import numpy as np


class PerceptionMixin:
    """Converts raw LiDAR ranges into robot-frame and world-frame Cartesian points."""

    def lidar2cartesian(self):
        # Convert Lidar polar coordinates to Cartesian coordinates
        ranges = np.array(self.lidar.getRangeImage())
        ranges[ranges == np.inf] = 100.0

        angles = np.linspace(
            2 / 3 * np.pi, -2 / 3 * np.pi, len(ranges)
        )  # Angles from 120 degrees to -120 degrees

        ranges = ranges[80:-80]
        angles = angles[80:-80]

        valid_mask = (ranges != np.inf) & (ranges < 5.0)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        X_lidar = np.array(
            [ranges * np.cos(angles), ranges * np.sin(angles), np.ones_like(ranges)]
        )

        r_X_l = np.array([[1, 0, 0.23], [0, 1, 0], [0, 0, 1]])

        w_T_r = np.array(
            [
                [np.cos(self.alpha), -np.sin(self.alpha), self.xw],
                [np.sin(self.alpha), np.cos(self.alpha), self.yw],
                [0, 0, 1],
            ]
        )

        X_robot = r_X_l @ X_lidar

        X_world = w_T_r @ X_robot

        return X_robot, X_world
