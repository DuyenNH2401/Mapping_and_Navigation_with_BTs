"""Robot base class — core initialization, devices, and motor control."""

import numpy as np
from controller import Supervisor


class RobotBase:
    """Core robot hardware: init, devices, step, and motor velocity control."""

    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.state = "forward"

        self.marker = self.robot.getFromDef("marker").getField("translation")

        self._init_constants()
        self._init_devices()

    # Devices

    def _init_devices(self):
        # Motors
        self.left_wheel_motor = self.robot.getDevice("wheel_left_joint")
        self.right_wheel_motor = self.robot.getDevice("wheel_right_joint")
        self.left_wheel_motor.setPosition(float("inf"))
        self.right_wheel_motor.setPosition(float("inf"))
        self.left_wheel_motor.setVelocity(0.0)
        self.right_wheel_motor.setVelocity(0.0)

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

    # Constants & state

    def _init_constants(self):
        self.max_velocity = 10.1523  # Maximum wheel velocity

        self.p_alpha = 3
        self.p_rho = np.pi

        self.is_finished = False

        self.xw = -0.1  # Robot's x position
        self.yw = 0.18  # Robot's y position
        self.alpha = 0.0  # Robot's orientation (yaw angle)

        self.map = np.zeros((300, 300))  # Initialize a 300x300 map with zeros

        self.waypoints = np.array(
            [
                # [-0.89, 0.75],
                [0.8, -0.2],
                [0.8, -0.45],
                [0.57, -1.83],
                [0.14, -3.14],
                [-0.9, -3.15],
                [-1.68, -2.92],
                [-1.68, -1.6],
                [-1.68, -0.09],
                [-0.74, 0.50],
            ]
        )

    # Simulation step

    def step(self):
        result = self.robot.step(self.timestep)
        if result == -1:
            return False
        return not self.is_finished

    # Motor control

    def set_wheel_velocity(self, left_velocity, right_velocity):
        left_velocity = np.clip(left_velocity, -self.max_velocity, self.max_velocity)
        right_velocity = np.clip(right_velocity, -self.max_velocity, self.max_velocity)

        self.left_wheel_motor.setVelocity(left_velocity)
        self.right_wheel_motor.setVelocity(right_velocity)
