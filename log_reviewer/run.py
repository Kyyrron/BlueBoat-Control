#!/usr/bin/env python3

r"""
BlueBoat log reviewer - entry point.

    python3 log_reviewer/run.py
    python3 log_reviewer/run.py ~/ros2_ws/data/Robot_data/<run>/<run>.csv
    python3 log_reviewer/run.py --no-fetch-tiles      # never touch the network

Runs under the system python3 and under ~/ros2_ws/.venv alike; it needs no
sourced ROS workspace, because nothing it imports touches rclpy.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Review, crop, replay and export a BlueBoat poslog CSV.")
    parser.add_argument("csv", nargs="?", help="a *-poslog.csv to open at start")
    parser.add_argument("--no-fetch-tiles", action="store_true",
                        help="draw only the satellite tiles already cached by the "
                             "Mission Control Station; never fetch")
    args = parser.parse_args(argv)

    from PySide6.QtWidgets import QApplication

    from reviewer.app import ReviewerWindow

    app = QApplication(sys.argv[:1])
    app.setApplicationName("BlueBoat log reviewer")
    window = ReviewerWindow(csv_path=args.csv, fetch_tiles=not args.no_fetch_tiles)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
