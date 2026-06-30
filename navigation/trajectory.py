"""Trajectory following — waypoint-based navigation with forward/backward pass."""

import numpy as np


class TrajectoryMixin:
    """Follows a list of waypoints, switching direction at the end."""

    def trajectory_following(self, index, waypoints):
        self._place_marker(waypoints, index)
        rho, alpha = self._compute_error(waypoints[index])

        phil = -self.p_alpha * alpha + self.p_rho * rho
        phir = self.p_alpha * alpha + self.p_rho * rho

        self.set_wheel_velocity(phil, phir)

        if rho < 0.3 and self.state == "Forward":
            index += 1
            if index >= len(waypoints):
                self.state = "Backward"
                index = len(waypoints) - 2

        elif rho < 0.3 and self.state == "Backward":
            index -= 1
            if index < 0:
                self.is_finished = True

        return index

    def _place_marker(self, waypoints, index):
        self.marker.setSFVec3f(
            [*waypoints[index], 0.0]
        )  # Set the marker's position in the world

    def _compute_error(self, point):
        # Compute the error between the robot's current position and the goal position
        dx = point[0] - self.xw
        dy = point[1] - self.yw

        rho = np.sqrt(dx**2 + dy**2)  # Distance to the goal
        alpha = np.arctan2(dy, dx) - self.alpha  # Angle to the goal

        # Normalize alpha to be within [-pi, pi]
        if alpha > np.pi:
            alpha -= 2 * np.pi
        elif alpha < -np.pi:
            alpha += 2 * np.pi

        return rho, alpha

    def follow_path(self, index, waypoints):
        """Follows a path defined by a list of waypoints."""
        if index >= len(waypoints):
            return index, True

        self._place_marker(waypoints, index)
        rho, alpha = self._compute_error(waypoints[index])

        phil = -self.p_alpha * alpha + self.p_rho * rho
        phir = self.p_alpha * alpha + self.p_rho * rho
        self.set_wheel_velocity(phil, phir)

        reached = False
        if rho < 0.3:
            index += 1
            if index >= len(waypoints):
                self.set_wheel_velocity(0.0, 0.0)
                reached = True

        return index, reached
