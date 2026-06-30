"""Mapping run script — per-step mapping and movement functions."""

import numpy as np
import scipy.signal as signal
from pathlib import Path

current_dir = Path(__file__).resolve().parent
target_dir = current_dir.parent / "map_save"
target_dir.mkdir(parents=True, exist_ok=True)
file_path = target_dir / "cspace.npy"
file_path_array = target_dir / "cspace_array.npy"


def mapping_run(robot):
    """One step of mapping: process LiDAR data and update probabilistic map."""
    X_world, X_robot = robot.lidar2cartesian()
    robot.probabilistic_mapping(X_world)
    x_pixel, y_pixel = robot.mapping(X_world[0], X_world[1])

    for x, y in zip(x_pixel, y_pixel):
        prob = robot.map[x, y]
        if prob > 0.1:
            v = int(prob * 255)
            color = int(v * 256**2 + v * 256 + v)
            robot.display.setColor(color)
            robot.display.drawPixel(x, y)

    rx, ry = robot.mapping(robot.xw, robot.yw)
    robot.display.setColor(0xFF0000)
    robot.display.drawPixel(rx, ry)


def save_cspace(robot):
    """Save configuration space map to file."""
    kernel = np.ones((21, 21))
    cmap = signal.convolve2d(robot.map, kernel, mode="same")
    cspace = cmap > 0.5
    np.save(file_path, cspace)
    np.save(file_path_array, robot.map)


def moving_table():
    """Legacy: standalone mapping loop (kept for reference)."""
    from tiago_robot import TiagoBT

    robot = TiagoBT()
    index = 1
    waypoints = robot.waypoints
    check = False
    while robot.step():
        robot.update_odometry()
        X_world, X_robot = robot.lidar2cartesian()

        index = robot.trajectory_following(index, waypoints)

        robot.probabilistic_mapping(X_world)
        x_pixel, y_pixel = robot.mapping(X_world[0], X_world[1])

        for x, y in zip(x_pixel, y_pixel):
            prob = robot.map[x, y]

            if prob > 0.1:
                v = int(prob * 255)
                color = int(v * 256**2 + v * 256 + v)
                robot.display.setColor(color)
                robot.display.drawPixel(x, y)

            rx, ry = robot.mapping(robot.xw, robot.yw)
            robot.display.setColor(0xFF0000)
            robot.display.drawPixel(rx, ry)

    if robot.state == "backward" and index == 0:
        check = True

    robot.set_wheel_velocity(0, 0)

    kernel = np.ones((20, 20))
    cmap = signal.convolve2d(robot.map, kernel, mode="same")
    cspace = cmap > 0.6

    np.save(file_path, cspace)

    if check:
        return True
    else:
        return False
