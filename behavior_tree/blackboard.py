"""Shared blackboard — global state for behavior-tree nodes."""


class Blackboard:
    """Simple key-value store with attribute-style access for convenience.

    Keys used by the system:
        robot      – TiagoBT instance
        path       – list of (xw, yw) waypoints to follow
        path_index – current waypoint index for MoveTo
    """

    def __init__(self):
        self.data = {}

    def write(self, key, value):
        self.data[key] = value

    def read(self, key):
        return self.data.get(key, None)
