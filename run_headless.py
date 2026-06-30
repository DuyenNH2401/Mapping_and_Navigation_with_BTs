#!/usr/bin/env python3
"""Run Webots simulation headless (no GUI).

Usage:
    python run_headless.py                  # default: behavior tree mode
    python run_headless.py --mode mapping   # mapping only mode
    python run_headless.py --mode navigation  # navigation only mode
    python run_headless.py --world worlds/kitchen.wbt  # custom world file

Modes:
    bt          - Full behavior tree: mapping → navigate to corner → navigate to sink
    mapping     - Mapping only: explore environment and save cspace.npy
    navigation  - Navigation only: load cspace.npy and navigate to goals
"""

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_WORLD = PROJECT_ROOT / "worlds" / "kitchen.wbt"


def main():
    parser = argparse.ArgumentParser(description="Run Webots headless")
    parser.add_argument(
        "--mode", choices=["bt", "mapping", "navigation"], default="bt",
        help="Simulation mode (default: bt)"
    )
    parser.add_argument(
        "--world", type=Path, default=DEFAULT_WORLD,
        help=f"Path to .wbt world file (default: {DEFAULT_WORLD})"
    )
    parser.add_argument(
        "--fast", action="store_true", default=True,
        help="Run in fast mode (default: True)"
    )
    parser.add_argument(
        "--no-fast", action="store_true", dest="realtime",
        help="Run in real-time mode instead of fast"
    )
    args = parser.parse_args()

    if not args.world.exists():
        print(f"ERROR: World file not found: {args.world}", file=sys.stderr)
        sys.exit(1)

    mode_flag = "fast" if not args.realtime else "realtime"

    cmd = [
        "webots",
        "--no-rendering",
        "--batch",
        "--stdout",
        "--stderr",
        f"--mode={mode_flag}",
        str(args.world),
    ]

    print(f"Launching: {' '.join(cmd)}")
    print(f"Mode: {args.mode}")
    print("-" * 60)

    env = {}
    if args.mode == "navigation":
        cspace_path = PROJECT_ROOT / "controllers" / "tiago_bt" / "map_save" / "cspace.npy"
        if not cspace_path.exists():
            print(
                f"ERROR: cspace.npy not found at {cspace_path}\n"
                f"       Run with --mode mapping first to generate the map.",
                file=sys.stderr,
            )
            sys.exit(1)
        print(f"Using map: {cspace_path}")

    process = subprocess.Popen(cmd)

    def signal_handler(sig, frame):
        print("\nReceived interrupt. Stopping simulation...")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print("Force killing...")
            process.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    exit_code = process.wait()
    print("-" * 60)
    if exit_code == 0:
        print("Simulation finished successfully.")
    else:
        print(f"Simulation exited with code {exit_code}")

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
