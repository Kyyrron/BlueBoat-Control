#!/usr/bin/env python3
"""
ROS-free tests of the run's path-update log (_custom_libraries/path_update_log.py)
and of its filing by poslog_report.finalise_run.

    /usr/bin/python3 test/test_path_update_log.py
"""

import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "blueboat_control", "src", "_custom_libraries"))

import numpy as np                          # noqa: E402

import path_update as pu                    # noqa: E402
import path_update_log as pl                # noqa: E402
import poslog_report as pr                  # noqa: E402
import robot_log_schema as rls              # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    if not cond:
        print(f"FAIL  {name}  {detail}")
        if "pytest" in sys.modules:
            raise AssertionError(f"{name}  {detail}")


def _events():
    """A realistic event stream from the core: cleared, refusal, first path,
    activation, update, end."""
    u = pu.PathUpdater()
    c = pu.Context(path_source="replanner", path_time=0.05, path_steps=2,
                   dt=0.05, run=3, boat=(0.1, -0.2, 0.3), ros_time=12.5,
                   wall_time=1.7e9, run_time=1.0)
    out = [u.reset(c, "run_started")]
    u.set_hold((0.0, 0.0, 0.0))
    t = np.arange(0.0, 40.0, 0.5)
    _, ev, _ = u.handle(pu.Request(id="far", t=list(t), x=list(0.5 * t + 3), y=[0.0] * len(t),
                                   yaw=[0.0] * len(t)), c)
    out.append(ev)
    _, ev, nt = u.handle(pu.Request(id="p0", t=list(t), x=list(0.5 * t), y=[0.0] * len(t),
                                    yaw=[0.0] * len(t)), c)
    out.append(ev)
    c.tau = nt
    out += u.tick(c)
    for tau in np.arange(0.0, 45.0, 0.5):
        c.tau = float(tau)
        out += u.tick(c)
    return out


def test_naming():
    check("pathlog name", pl.pathlog_path("/d/2026_10_15-10_00_00-sim-poslog.csv")
          == "/d/2026_10_15-10_00_00-sim-pathlog.csv")
    check("pathlog name keeps the -N suffix",
          pl.pathlog_path("/d/2026_10_15-10_00_00-sim-poslog-2.csv")
          == "/d/2026_10_15-10_00_00-sim-pathlog-2.csv")
    name = os.path.basename(pl.pathlog_path("/d/2026_10_15-10_00_00-sim-poslog.csv"))
    check("never matched by the reviewer smoke test's '-poslog' filter",
          "-poslog" not in name and name.endswith(".csv"))


def test_legend():
    check("legend covers every column", set(pl.LEGEND) == set(pl.COLUMNS))
    check("no comma in any description or unit",
          all("," not in d and "," not in u for d, u in pl.LEGEND.values()))
    check("descriptions are 1-4 words",
          all(1 <= len(d.split()) <= 4 for d, _ in pl.LEGEND.values()),
          [d for d, _ in pl.LEGEND.values() if not 1 <= len(d.split()) <= 4])
    check("ASCII legend", all(d.isascii() and u.isascii() for d, u in pl.LEGEND.values()))
    check("distinct from the poslog schema's columns",
          not (set(pl.COLUMNS) & set(rls.COLUMNS_PINGER) - {"run"}))


def test_lazy_creation_and_rows():
    events = _events()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "x-pathlog.csv")
        log = pl.PathLog(path)
        log.write_event(events[0])                      # 'cleared' only
        check("no file before the first request event", not os.path.exists(path))
        for ev in events[1:]:
            log.write_event(ev)
        log.close()
        check("file created by the first request", os.path.isfile(path))
        desc, units, rows = pl.read_pathlog(path)
        check("3 header rows: description, unit, names",
              desc == [pl.LEGEND[c][0] for c in pl.COLUMNS]
              and units == [pl.LEGEND[c][1] for c in pl.COLUMNS])
        kinds = [r["event"] for r in rows]
        check("held 'cleared' written first, then every event in order",
              kinds == ["cleared", "request_refused", "request_accepted", "activated",
                        "end_reached"], kinds)
        check("one row per event", log.rows == len(events))
        refused = rows[1]
        check("refusal row: id, reason, gap",
              refused["update_id"] == "far" and refused["reason"] == pu.DISCONTINUOUS_POSITION
              and math.isclose(float(refused["pos_gap"]), 3.0))
        check("boat pose and clocks recorded",
              rows[2]["boat_x"] == "0.1" and rows[2]["ros_time"] == "12.5"
              and rows[2]["run"] == "3")
        check("every row has every column", all(len(r) == len(pl.COLUMNS) for r in rows))
        # never overwrite (CM-7)
        try:
            pl.PathLog(path).write_event(events[1])
            overwrote = True
        except FileExistsError:
            overwrote = False
        check("an existing pathlog is never overwritten", not overwrote)


def test_no_request_no_file():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "y-pathlog.csv")
        log = pl.PathLog(path)
        for ev in _events()[:1]:
            log.write_event(ev)
        log.close()
        check("a run without any request leaves no pathlog", not os.path.exists(path))
        check("writes after close are ignored", log.write_event(_events()[1]) is False)


def test_free_text_sanitised():
    ev = _events()[1]
    ev = dict(ev, message="a, b\nc")
    row = pl.row_from_event(ev)
    check("commas and newlines removed from free text",
          "," not in row["message"] and "\n" not in row["message"])
    check("None -> empty cell, bool -> 0/1",
          pl.row_from_event(dict(ev, lead_min=None))["lead_min"] == ""
          and pl.row_from_event(dict(ev, heading_checked=True))["heading_checked"] == "1")


def test_filing():
    """poslog_report.finalise_run moves the pathlog into the run folder, and
    behaves exactly as before when there is none."""
    with tempfile.TemporaryDirectory() as d:
        for with_log in (True, False):
            stem = f"2026_10_15-10_00_0{int(with_log)}-sim-poslog"
            csv = os.path.join(d, stem + ".csv")
            with open(csv, "w") as h:
                h.write("x\n")
            if with_log:
                log = pl.PathLog(pl.pathlog_path(csv))
                for ev in _events():
                    log.write_event(ev)
                log.close()
            folder = pr.finalise_run(csv)
            names = sorted(os.listdir(folder))
            want = os.path.basename(pl.pathlog_path(csv))
            if with_log:
                check("pathlog filed into the run folder", want in names, names)
                check("nothing left flat", not os.path.exists(pl.pathlog_path(csv)))
            else:
                check("no pathlog: the folder holds what it always did",
                      want not in names and stem + ".csv" in names, names)


TESTS = [test_naming, test_legend, test_lazy_creation_and_rows, test_no_request_no_file,
         test_free_text_sanitised, test_filing]


def main():
    for test in TESTS:
        test()
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
