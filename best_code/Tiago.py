"""Tiago controller."""

import matplotlib.pyplot as plt
import numpy as np
from controller import Supervisor
from scipy import signal


class TiagoRobot:
    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.is_finished = False
        self.state = "forward"

        # Initialize Odometry state
        self.xw = -0.1
        self.yw = 0.18
        self.theta = 0.0

        # Initialize P gains
        self.p_alpha = 3
        self.p_rho = np.pi * 3

        # Mapping
        self.map = np.zeros((300, 300))

        # Initialize constants
        self._init_constants()

        # Initialize devices
        self._init_devices()

        # Marker
        self.marker = self.robot.getFromDef("marker").getField("translation")

    def _init_constants(self):
        self.MAX_SPEED = 10.1523
        self.WHEEL_RADIUS = 0.016
        self.BASE_SPEED = 0.5 * self.MAX_SPEED

    def _init_devices(self):
        # Motors
        self.left_motor = self.robot.getDevice("wheel_left_joint")
        self.right_motor = self.robot.getDevice("wheel_right_joint")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Lidar
        self.lidar = self.robot.getDevice("Hokuyo URG-04LX-UG01")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        # Display
        self.display = self.robot.getDevice("display")

        # Compass
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)

        # GPS
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

    def step_simulation(self):
        if self.robot.step(self.timestep) == -1:
            return False
        return not self.is_finished

    def set_motor_speeds(self, left_speed, right_speed):
        left_speed = np.clip(left_speed, -self.MAX_SPEED, self.MAX_SPEED)
        right_speed = np.clip(right_speed, -self.MAX_SPEED, self.MAX_SPEED)

        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def update_odometry(self):
        self.xw = self.gps.getValues()[0]
        self.yw = self.gps.getValues()[1]
        self.theta = np.arctan2(
            self.compass.getValues()[0], self.compass.getValues()[1]
        )

    def lidar2cartesian(self):
        ranges = np.array(self.lidar.getRangeImage())
        ranges[ranges == np.inf] = 100.0

        angles = np.linspace(2 / 3 * np.pi, -2 / 3 * np.pi, len(ranges))

        ranges = ranges[80:-80]
        angles = angles[80:-80]

        valid_mask = (ranges != np.inf) & (ranges < 5.0)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        w_T_r = np.array(
            [
                [np.cos(self.theta), -np.sin(self.theta), self.xw],
                [np.sin(self.theta), np.cos(self.theta), self.yw],
                [0, 0, 1],
            ]
        )

        X_lidar = np.array(
            [np.cos(angles) * ranges, np.sin(angles) * ranges, np.ones(len(ranges))]
        )

        r_X_l = np.array(
            [
                [1, 0, 0.23],
                [0, 1, 0],
                [0, 0, 1],
            ]
        )
        X_robot = r_X_l @ X_lidar

        X_world = w_T_r @ X_robot

        return X_world, X_robot

    def trajectory_following(self, index, waypoints):
        # self._place_marker(index, waypoints)
        rho, alpha = self._compute_error(waypoints[index])

        # Alpha dương thì xoay trái, alpha âm thì xoay phải
        phil = -alpha * self.p_alpha + rho * self.p_rho
        phir = alpha * self.p_alpha + rho * self.p_rho

        self.set_motor_speeds(phil, phir)

        if self.state == "forward" and rho < 0.3:
            index += 1
            if index >= len(waypoints):
                self.state = "backward"
                index = len(waypoints) - 2

        elif self.state == "backward" and rho < 0.3:
            index -= 1
            if index < 0:
                self.is_finished = True

        return index

    def _compute_error(self, point):
        delta_x = point[0] - self.xw
        delta_y = point[1] - self.yw

        rho = np.sqrt(delta_x**2 + delta_y**2)
        alpha = np.arctan2(delta_y, delta_x) - self.theta

        if alpha > np.pi:
            alpha -= 2 * np.pi
        elif alpha < -np.pi:
            alpha += 2 * np.pi

        return rho, alpha

    def _place_marker(self, index, waypoints):
        self.marker.setSFVec3f([*waypoints[index], 0.0])

    def mapping(self, xw, yw):
        X_max, X_min = +2.25, -2.25  # +1.23, -2.23
        Y_max, Y_min = +1.75, -3.92  # +1.72, -3.82

        x_val = (xw - X_min) / (X_max - X_min) * 300
        y_val = (yw - Y_min) / (Y_max - Y_min) * 300

        if isinstance(xw, np.ndarray):
            x_pixel = np.clip(x_val.astype(int), 0, 299)
            y_pixel = np.clip(y_val.astype(int), 0, 299)

        else:
            x_pixel = max(0, min(299, int(x_val)))
            y_pixel = max(0, min(299, int(y_val)))

        return x_pixel, y_pixel

    def probabilistic_mapping(self, X_world):
        x_pixel, y_pixel = self.mapping(X_world[0], X_world[1])
        np.add.at(self.map, (x_pixel, y_pixel), 0.005)
        self.map = np.clip(self.map, 0.0, 1.0)


# Main loop:
def main(mode="mapping"):
    # waypoints = np.array(
    #     [
    #         # [-0.89, 0.75],
    #         [-0.04, 0.3],
    #         [0.67, -0.45],
    #         [0.57, -1.83],
    #         [0.14, -3.14],
    #         [-0.78, -3.15],
    #         [-1.8, -2.92],
    #         [-1.8, -1.6],
    #         [-1.77, -0.09],
    #         [-0.74, 0.50],
    #     ]
    # )

    robot = TiagoRobot()
    index = 1
    if mode == "mapping":
        while robot.step_simulation():
            robot.update_odometry()
            X_world, _ = robot.lidar2cartesian()

            index = robot.trajectory_following(index, waypoints)
            robot.probabilistic_mapping(X_world)
            x_pixel, y_pixel = robot.mapping(X_world[0], X_world[1])

            for x, y in zip(x_pixel.tolist(), y_pixel.tolist()):
                prob = robot.map[x, y]
                # prob = 1.0 if raw_prob > 0.7 else 0
                if prob > 0.1:
                    v = int(prob * 255)
                    color = int(v * 256**2 + v * 256 + v)
                    robot.display.setColor(color)
                    robot.display.drawPixel(x, y)

            rx, ry = robot.mapping(robot.xw, robot.yw)
            robot.display.setColor(0xFF0000)
            robot.display.drawPixel(rx, ry)

        robot.set_motor_speeds(0, 0)

        kernel = np.ones((20, 20))
        cmap = signal.convolve2d(robot.map, kernel, mode="same")
        cspace = cmap > 0.6

        # plt.imshow(cspace)
        # plt.show()

        np.save("cspace", cspace)

    elif mode == "navigation":
        from A_star import a_star

        goal = (-1, -3.4)
        grid_map = np.load("cspace.npy")
        start_pixel = robot.mapping(robot.xw, robot.yw)
        goal_pixel = robot.mapping(goal[0], goal[1])

        path_pixel, distance = a_star(grid_map, start=start_pixel, goal=goal_pixel)

        robot.display.setColor(0xFFFFFF)
        for x in range(300):
            for y in range(300):
                if grid_map[x, y]:
                    robot.display.drawPixel(x, y)

        def inverse_mapping(x_pixel, y_pixel):
            X_max, X_min = +2.25, -2.25
            Y_max, Y_min = +1.75, -3.92

            xw = (x_pixel / 300.0) * (X_max - X_min) + X_min
            yw = (y_pixel / 300.0) * (Y_max - Y_min) + Y_min
            return xw, yw

        path_world = [inverse_mapping(x, y) for x, y in path_pixel]

        robot.display.setColor(0x00FF00)
        for node in path_pixel:
            robot.display.drawPixel(node[0], node[1])

        waypoints = np.array(path_world)

        robot.display.setColor(0xFFFF00)
        robot.display.drawPixel(goal_pixel[0], goal_pixel[1])

        while robot.step_simulation():
            robot.update_odometry()
            if len(waypoints) > 0:
                index = robot.trajectory_following(index, waypoints)
            rx, ry = robot.mapping(robot.xw, robot.yw)
            robot.display.setColor(0xFF0000)
            robot.display.drawPixel(rx, ry)

        robot.set_motor_speeds(0, 0)


if __name__ == "__main__":
    main(mode="navigation")
