"""TIAGo behavior-tree controller — main entry point."""

import py_trees
from py_trees.composites import Parallel, Selector, Sequence

from behavior_tree.blackboard import Blackboard
from behavior_tree.check_map import MapExist
from behavior_tree.map_running import MoveTable, RunMapping
from behavior_tree.navigation_nodes import ComputePath, MoveTo
from tiago_robot import TiagoBT

LEFT_CORNER = (-1.4, -3.0)
SINK_POINT = (0.75, 0.75)


def create_behavior_tree(blackboard):
    check_map_selector = Selector(name="Does Map Exist?", memory=False)
    test_map = MapExist(name="Test for map")

    mapping_parallel = Parallel(
        name="Mapping", policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    map_env = RunMapping(name="Mapping environment", blackboard=blackboard)
    move_table = MoveTable(name="Move around the table", blackboard=blackboard)
    mapping_parallel.add_children([map_env, move_table])

    check_map_selector.add_children([test_map, mapping_parallel])

    compute_path_llc = ComputePath(
        name="Path to corner", point=LEFT_CORNER, blackboard=blackboard
    )
    move_to_llc = MoveTo(name="Go to corner", blackboard=blackboard)

    compute_path_sink = ComputePath(
        name="Path to sink", point=SINK_POINT, blackboard=blackboard
    )
    move_to_sink = MoveTo(name="Go to sink", blackboard=blackboard)

    root = Sequence(name="Main", memory=True)
    root.add_children(
        [
            check_map_selector,
            compute_path_llc,
            move_to_llc,
            compute_path_sink,
            move_to_sink,
        ]
    )

    return root


def main():
    robot = TiagoBT()
    blackboard = Blackboard()
    blackboard.write("robot", robot)

    root = create_behavior_tree(blackboard)
    tree = py_trees.trees.BehaviourTree(root)
    print("Behavior tree created. Starting main loop...")

    while robot.step():
        robot.update_odometry()
        tree.tick()

        # Debug: print feedback from all running/failed nodes
        def _print_feedback(node, depth=0):
            msg = getattr(node, "feedback_message", None)
            if msg:
                print(f"{'  ' * depth}[{node.name}] {msg}")
            for child in getattr(node, "children", []):
                _print_feedback(child, depth + 1)

        _print_feedback(tree.root)

        if tree.root.status == py_trees.common.Status.SUCCESS:
            print("Mission complete!")
            break
        if tree.root.status == py_trees.common.Status.FAILURE:
            print("Mission failed!")
            break

    robot.set_wheel_velocity(0.0, 0.0)


if __name__ == "__main__":
    main()
