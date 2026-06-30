"""TiagoBT — assembled robot controller."""

from tiago_robot.base import RobotBase
from tiago_robot.odometry import Odometry
from tiago_robot.perception import PerceptionMixin
from navigation.trajectory import TrajectoryMixin
from mapping.grid_map import MappingMixin


class TiagoBT(RobotBase, Odometry, PerceptionMixin, TrajectoryMixin, MappingMixin):
    """TIAGo robot controller with odometry, perception, navigation, and mapping."""

    pass
