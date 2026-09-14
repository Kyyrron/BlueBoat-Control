"""BlueBoat log reviewer - an interactive desktop reader for poslog CSVs.

Standalone PySide6 application. NOT a ROS package, not built by colcon (the
folder carries a COLCON_IGNORE). It is strictly READ-ONLY against
`~/ros2_ws/data/Robot_data/`, which is primary field record (superproject CM-7
/ BlueBoat-Control N7); everything it writes goes under
`~/ros2_ws/data/Processed_Robot_data/`.
"""

__all__ = ["poslog_bridge", "tiles", "figures", "track_view", "timeline",
           "export", "app"]
