#!/usr/bin/env python3
"""
ROS-free tests of how a path ENDS (R10, 2026-11-24).

A non-loop path clamps at its final pose (position AND heading) for every
t > duration, so the controllers hold it until STOP RUN; a loop path wraps.
The same holds for the replanner's in-memory paths (YamlTrajectory.from_points,
loop False). And the designer's "every path loops" report was the mission files'
`loop: true`, not this sampler (docs/PINGER_FILTER_FINDINGS.md section 8).

    /usr/bin/python3 test/test_yaml_trajectory_end.py
"""

import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "blueboat_control", "src", "_custom_libraries"))

import yaml_trajectory as yt                # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    if not cond:
        print(f"FAIL  {name}  {detail}")
        if "pytest" in sys.modules:
            raise AssertionError(f"{name}  {detail}")


def _points():
    # east 10 m at 0.5 m/s, then a 90 deg turn north for 4 m: ends at (10, 4), heading north
    pts = [[0.5 * k, 0.25 * k, 0.0, 0.0] for k in range(41)]           # t 0..20 s
    pts += [[20.0 + 0.5 * k, 10.0, 0.25 * k, math.pi / 2] for k in range(1, 17)]   # t ..28 s
    return pts


def _write(path, loop):
    with open(path, "w") as fh:
        fh.write("format: blueboat_trajectory/1\nname: end_test\nloop: %s\npoints:\n" %
                 ("true" if loop else "false"))
        for p in _points():
            fh.write("- [%r, %r, %r, %r]\n" % tuple(p))


def test_non_loop_clamps():
    with tempfile.TemporaryDirectory() as tmp:
        f = os.path.join(tmp, "p.yaml")
        _write(f, loop=False)
        tr = yt.YamlTrajectory(f)
        check("duration read from the last sample", abs(tr.duration - 28.0) < 1e-12, tr.duration)
        end = tr.pose(tr.duration)
        for t in (28.0001, 30.0, 60.0, 600.0, 1e6):
            p = tr.pose(t)
            check(f"t = {t:g} s > duration: final position held", (p[0], p[1]) == (end[0], end[1]), p)
            check(f"t = {t:g} s > duration: final heading held", p[5] == end[5], p[5])
        check("final pose is the last sample", abs(end[0] - 10.0) < 1e-12 and abs(end[1] - 4.0) < 1e-12
              and abs(end[5] - math.pi / 2) < 1e-12, end)
        check("never back to the start after the end", all(tr.pose(28.0 + k)[0] == 10.0 for k in range(100)))


def test_loop_wraps():
    with tempfile.TemporaryDirectory() as tmp:
        f = os.path.join(tmp, "p.yaml")
        _write(f, loop=True)
        tr = yt.YamlTrajectory(f)
        for t in (5.0, 13.0, 25.0):
            a, b = tr.pose(t), tr.pose(t + tr.duration)
            check(f"loop: pose(t + duration) == pose(t) at t = {t}", abs(a[0] - b[0]) < 1e-9
                  and abs(a[1] - b[1]) < 1e-9, (a, b))
        check("loop: back at the start one duration later", abs(tr.pose(tr.duration + 1e-9)[0]) < 1e-6)


def test_from_points():
    pts = _points()
    tr = yt.YamlTrajectory.from_points(pts, name="replanner")
    check("from_points defaults to loop False", tr.loop is False)
    end = tr.pose(tr.duration)
    check("from_points clamps like a file", tr.pose(tr.duration + 50.0) == end)
    trl = yt.YamlTrajectory.from_points(pts, loop=True)
    check("from_points loop=True wraps", abs(trl.pose(trl.duration + 5.0)[0] - trl.pose(5.0)[0]) < 1e-9)


def test_path_info_ends():
    """path_generation's /blueboat/path_info end times of the library shapes
    (the shapes that hold their last pose), read without ROS."""
    src = open(os.path.join(HERE, "..", "blueboat_control", "src", "_custom_libraries",
                            "path_generation.py")).read()
    check("path_info declares the library shapes' ends",
          "SHAPE_END_S = {'sin': 500.0, 'kin_square': 500.0, 'seabed_scanning': 40.0 + 12.0 * math.pi}" in src)


TESTS = [test_non_loop_clamps, test_loop_wraps, test_from_points, test_path_info_ends]


def main():
    for test in TESTS:
        test()
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
