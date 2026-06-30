import py_trees
import os
from pathlib import Path

CSPACE_PATH = Path(__file__).resolve().parent.parent / "map_save" / "cspace.npy"


class MapExist(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        print(f"Execute: {self.name}")
        if CSPACE_PATH.exists():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
