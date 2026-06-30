"""Behavior-tree nodes for path planning and movement."""

import py_trees
from navigation.pathfinding import load_cspace, astar


class ComputePath(py_trees.behaviour.Behaviour):
    """Compute a path from the robot's current position to a goal point."""

    def __init__(self, name, point, blackboard):
        super().__init__(name)
        self.point = point
        self.blackboard = blackboard

    def update(self):

        robot = self.blackboard.read("robot")
        if robot is None:
            return py_trees.common.Status.FAILURE

        cspace = load_cspace()
        if cspace is None:
            self.feedback_message = "cspace.npy not found — run mapping first"
            return py_trees.common.Status.FAILURE

        from navigation.pathfinding import world_to_pixel

        rows, cols = cspace.shape
        start = (robot.xw, robot.yw)
        start_px = world_to_pixel(start[0], start[1], rows, cols)
        end_px = world_to_pixel(self.point[0], self.point[1], rows, cols)

        cspace[start_px[0], start_px[1]] = False
        cspace[end_px[0], end_px[1]] = False
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                nx = start_px[0] + dx
                ny = start_px[1] + dy
                if 0 <= nx < rows and 0 <= ny < cols:
                    cspace[nx, ny] = False

        path = astar(cspace, start, self.point)

        if path is None:
            self.feedback_message = f"No path from {start} to {self.point}"
            return py_trees.common.Status.FAILURE

        self.blackboard.write("path", path)
        self.blackboard.write("path_index", 0)
        self.feedback_message = f"Path: {len(path)} waypoints"
        return py_trees.common.Status.SUCCESS


class MoveTo(py_trees.behaviour.Behaviour):
    """Move the robot along a path stored on the blackboard."""

    def __init__(self, name, blackboard):
        super().__init__(name)
        self.blackboard = blackboard

    def update(self):

        robot = self.blackboard.read("robot")
        if robot is None:
            return py_trees.common.Status.FAILURE

        path = self.blackboard.read("path")
        if path is None:
            self.feedback_message = "no path on blackboard"
            return py_trees.common.Status.FAILURE

        index = self.blackboard.read("path_index") or 0

        index, reached = robot.follow_path(index, path)
        self.blackboard.write("path_index", index)

        if reached:
            self.feedback_message = "arrived"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"waypoint {index}/{len(path)}"
        return py_trees.common.Status.RUNNING
