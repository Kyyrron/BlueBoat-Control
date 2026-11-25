#!/usr/bin/env python3
"""Runtime loader for designer-generated YAML trajectories.

Used by ``path_generation.py``; no dependency on the Mission Control
Station, only PyYAML and numpy.

The file format (``blueboat_trajectory/1``, see
``docs/05_trajectory_format.md`` in the station repository) stores a dense
list of time-stamped samples ``[t, x, y, yaw]``. Evaluation at an arbitrary
time is a binary search plus linear interpolation (yaw interpolated with
wrap-around), clamped at the final pose — matching the "default to last
known point" convention of the existing hard-coded trajectories — or wrapped
when ``loop: true``.
"""

from __future__ import annotations

import math

import numpy as np
import yaml

SUPPORTED_FORMAT = "blueboat_trajectory/1"


class YamlTrajectory:
    """A time-parameterized trajectory loaded from a designer YAML file."""

    def __init__(self, path: str) -> None:
        with open(path, "r") as handle:
            # The C parser when PyYAML has it: the same objects, ~5x faster on
            # a long mission (parsed inside the /path_request callback).
            data = yaml.load(handle, Loader=getattr(yaml, "CSafeLoader", yaml.SafeLoader)) or {}
        fmt = data.get("format", "")
        if fmt != SUPPORTED_FORMAT:
            raise ValueError(
                f"Unsupported trajectory format '{fmt}' in {path} "
                f"(expected {SUPPORTED_FORMAT})")
        self._set(data.get("points", []), data.get("name", ""), data.get("loop", False), f" in {path}")

    @classmethod
    def from_points(cls, points, name: str = "", loop: bool = False) -> "YamlTrajectory":
        """Build a trajectory from ``[t, x, y, yaw]`` rows already in memory:
        the samples of a ``/blueboat/update_path`` request are evaluated by THIS
        class, so an updated path is followed exactly as a designer file
        holding the same samples would be."""
        self = cls.__new__(cls)
        self._set(points, name, loop)
        return self

    def _set(self, points, name, loop, where=""):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 4 or len(points) == 0:
            raise ValueError(f"No valid points{where}")
        self.name: str = str(name)
        self.loop: bool = bool(loop)
        self.duration: float = float(points[-1, 0])
        self._t = points[:, 0]
        self._x = points[:, 1]
        self._y = points[:, 2]
        self._yaw = points[:, 3]

    def pose(self, t: float) -> tuple[float, float, float, float, float, float]:
        """Return ``(x, y, z, roll, pitch, yaw)`` at time ``t`` seconds."""
        if self.loop and self.duration > 0:
            t = t % self.duration
        if t <= self._t[0]:
            i0 = i1 = 0
            u = 0.0
        elif t >= self._t[-1]:
            i0 = i1 = len(self._t) - 1
            u = 0.0
        else:
            i1 = int(np.searchsorted(self._t, t, side="right"))
            i0 = i1 - 1
            span = self._t[i1] - self._t[i0]
            u = (t - self._t[i0]) / span if span > 0 else 0.0
        x = self._x[i0] + u * (self._x[i1] - self._x[i0])
        y = self._y[i0] + u * (self._y[i1] - self._y[i0])
        dyaw = _wrap(self._yaw[i1] - self._yaw[i0])
        yaw = _wrap(self._yaw[i0] + u * dyaw)
        return float(x), float(y), 0.0, 0.0, 0.0, float(yaw)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

