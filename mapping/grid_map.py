"""Grid mapping — world-to-pixel coordinate transforms and probabilistic occupancy map."""

import numpy as np


class MappingMixin:
    """Coordinate transforms between world frame and grid-map pixels,
    plus probabilistic mapping updates."""

    def mapping(self, xw, yw):
        X_max, X_min = +2.25, -2.25  # +1.23, -2.23
        Y_max, Y_min = +1.75, -3.92  # +1.72, -3.82

        rows, cols = self.map.shape  # (200, 300)

        x_val = (xw - X_min) / (X_max - X_min) * cols
        y_val = (yw - Y_min) / (Y_max - Y_min) * rows

        if isinstance(xw, np.ndarray):
            x_pixel = np.clip(x_val.astype(int), 0, cols - 1)
            y_pixel = np.clip(y_val.astype(int), 0, rows - 1)

        else:
            x_pixel = max(0, min(cols - 1, int(x_val)))
            y_pixel = max(0, min(rows - 1, int(y_val)))

        return x_pixel, y_pixel

    def inverse_mapping(self, x_pixel, y_pixel):
        X_max, X_min = +2.25, -2.25
        Y_max, Y_min = +1.75, -3.92

        xw = (x_pixel / 300.0) * (X_max - X_min) + X_min
        yw = (y_pixel / 300.0) * (Y_max - Y_min) + Y_min
        return xw, yw

    def probabilistic_mapping(self, X_world):
        x_pixel, y_pixel = self.mapping(X_world[0], X_world[1])
        np.add.at(self.map, (y_pixel, x_pixel), 0.05)
        self.map = np.clip(self.map, 0.0, 1.0)
