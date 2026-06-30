"""Behavior-tree nodes for mapping phase."""

import py_trees

from mapping.move_table import mapping_run, save_cspace


class RunMapping(py_trees.behaviour.Behaviour):
    """Process LiDAR data and update probabilistic map each tick."""

    def __init__(self, name, blackboard):
        super(RunMapping, self).__init__(name)
        self.blackboard = blackboard

    def update(self):
        robot = self.blackboard.read("robot")
        if robot is None:
            return py_trees.common.Status.FAILURE

        mapping_run(robot)
        return py_trees.common.Status.RUNNING


class MoveTable(py_trees.behaviour.Behaviour):
    """Move the robot around the table following waypoints to explore.

    Returns SUCCESS when the forward+backward trajectory pass is complete.
    """

    def __init__(self, name, blackboard):
        super(MoveTable, self).__init__(name)
        self.blackboard = blackboard
        self.index = 1
        self.initialised = False

    def initialise(self):
        if not self.initialised:
            robot = self.blackboard.read("robot")
            if robot is not None:
                self.index = 1
                self.waypoints = robot.waypoints
                self.initialised = True

    def update(self):
        robot = self.blackboard.read("robot")
        if robot is None:
            return py_trees.common.Status.FAILURE

        self.index = robot.trajectory_following(self.index, self.waypoints)

        if robot.state == "Backward" and self.index == 0:
            save_cspace(robot)
            robot.set_wheel_velocity(0.0, 0.0)
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
