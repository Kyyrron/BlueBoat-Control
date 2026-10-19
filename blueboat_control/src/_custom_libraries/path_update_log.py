#!/usr/bin/env python3
"""
The run's path-update log: `<date>-<note>-pathlog.csv`, beside the poslog.

ROS-FREE BY CONSTRUCTION (standard library only), like robot_log_schema and
poslog_report, so the format is checked from a plain Python prompt.

One row per /blueboat/path_update_event message -- every request with its
answer, every segment activation, the end of the path, every clear -- so a run
can be read back afterwards to see exactly which path was active when
(docs/PATH_UPDATE_DESIGN.md, traceability).

Same writing rules as the poslog (robot_log_schema):
  * row 1: a short description of each column, row 2: its unit, row 3: the
    column names, data from row 4 -- all three written AT FILE CREATION, never
    at run end, so a killed run keeps them;
  * rows are filled BY COLUMN NAME;
  * no description, unit or value contains a comma (free text is sanitised),
    because the writers join with ',' and nothing is quoted.

Written by robot_interface (real) and simulation_interface (sim), the nodes
that own the run's poslog, and filed into the run's folder by
poslog_report.finalise_run. NEVER created for a run that received no request:
the file only comes into existence with the first request event. Transition
events seen before it ('cleared' at a run start) are held in memory and
written then, so a replanner run nobody sent a path to leaves no file. A pinger
run gets no PathLog at all.
"""

import os

from poslog_report import pathlog_path      # the naming rule, owned by the filer

__all__ = ["COLUMNS", "LEGEND", "PathLog", "pathlog_path", "row_from_event", "exists_for",
           "read_pathlog", "REQUEST_EVENTS"]

REQUEST_EVENTS = ("request_accepted", "request_refused")

# Column order is part of the record: append only, never reorder (CM-7).
COLUMNS = (
    "ros_time", "wall_time", "run_time", "run", "seq", "event", "update_id",
    "reason", "change_point", "tau", "lead_min", "points", "t_first", "t_last",
    "x_first", "y_first", "yaw_first", "pos_gap", "heading_gap",
    "heading_checked", "boat_x", "boat_y", "boat_psi", "active_id",
    "handling_ms", "message",
)

# name -> (description, unit). 1-4 words, ASCII, no commas.
LEGEND = {
    "ros_time": ("Node clock time", "s"),
    "wall_time": ("Wall clock", "s (unix)"),
    "run_time": ("Time since run start", "s"),
    "run": ("Run number", "-"),
    "seq": ("Event counter", "-"),
    "event": ("Event kind", "-"),
    "update_id": ("Update identifier", "-"),
    "reason": ("Answer code", "-"),
    "change_point": ("Change point", "s (path)"),
    "tau": ("Path parameter tau", "s (path)"),
    "lead_min": ("Minimum lead", "s (path)"),
    "points": ("Sample count", "-"),
    "t_first": ("First sample time", "s (path)"),
    "t_last": ("Last sample time", "s (path)"),
    "x_first": ("First sample x", "m"),
    "y_first": ("First sample y", "m"),
    "yaw_first": ("First sample yaw", "rad"),
    "pos_gap": ("Junction position gap", "m"),
    "heading_gap": ("Junction heading gap", "rad"),
    "heading_checked": ("Heading gap enforced", "0/1"),
    "boat_x": ("Boat x", "m"),
    "boat_y": ("Boat y", "m"),
    "boat_psi": ("Boat heading", "rad"),
    "active_id": ("Path owner after event", "-"),
    "handling_ms": ("Request handling time", "ms"),
    "message": ("Human readable detail", "-"),
}

_PENDING_LIMIT = 50


def _clean(text):
    """Free text for an unquoted CSV cell: no comma, no line break."""
    return (str(text).replace(",", ";").replace("\r", " ")
            .replace("\n", " ").strip())


def _cell(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, int):
        return str(value)
    return _clean(value)


def row_from_event(event):
    """The pathlog row (column -> cell text) of one event dict."""
    boat = event.get("boat") or {}
    values = {
        "ros_time": event.get("ros_time"),
        "wall_time": event.get("wall_time"),
        "run_time": event.get("run_time"),
        "run": event.get("run"),
        "seq": event.get("seq"),
        "event": event.get("event"),
        "update_id": event.get("id"),
        "reason": event.get("reason"),
        "change_point": event.get("change_point"),
        "tau": event.get("tau"),
        "lead_min": event.get("lead_min"),
        "points": event.get("points"),
        "t_first": event.get("t_first"),
        "t_last": event.get("t_last"),
        "x_first": event.get("x_first"),
        "y_first": event.get("y_first"),
        "yaw_first": event.get("yaw_first"),
        "pos_gap": event.get("pos_gap"),
        "heading_gap": event.get("heading_gap"),
        "heading_checked": event.get("heading_checked"),
        "boat_x": boat.get("x"),
        "boat_y": boat.get("y"),
        "boat_psi": boat.get("psi"),
        "active_id": event.get("active_id"),
        "handling_ms": event.get("handling_ms"),
        "message": event.get("message"),
    }
    return {c: _cell(values[c]) for c in COLUMNS}


class PathLog:
    """Lazy writer of one run's pathlog (see the module docstring)."""

    def __init__(self, path):
        self.path = path
        self._file = None
        self._pending = []
        self.rows = 0
        self.closed = False

    @property
    def created(self):
        return self._file is not None or (self.rows > 0)

    def write_event(self, event):
        """Record one event. Returns True when a row reached the file."""
        if self.closed:
            return False
        if self._file is None:
            if event.get("event") not in REQUEST_EVENTS:
                self._pending.append(event)
                del self._pending[:-_PENDING_LIMIT]
                return False
            self._open()
            for held in self._pending:
                self._write(held)
            self._pending = []
        self._write(event)
        return True

    def _open(self):
        # 'x': create, never overwrite -- a field record is write-once (CM-7).
        self._file = open(self.path, "x", buffering=1)
        self._file.write(",".join(LEGEND[c][0] for c in COLUMNS) + "\n")
        self._file.write(",".join(LEGEND[c][1] for c in COLUMNS) + "\n")
        self._file.write(",".join(COLUMNS) + "\n")
        self._file.flush()

    def _write(self, event):
        row = row_from_event(event)
        self._file.write(",".join(row[c] for c in COLUMNS) + "\n")
        self._file.flush()
        self.rows += 1

    def close(self):
        """Close the file (if it was ever created). Idempotent."""
        self.closed = True
        self._pending = []
        if self._file is not None and not self._file.closed:
            self._file.flush()
            self._file.close()


def read_pathlog(path):
    """Read a pathlog back: (descriptions, units, rows), rows a list of dicts
    column -> text. For tests and offline analysis."""
    with open(path, "r") as handle:
        lines = [line.rstrip("\n") for line in handle]
    if len(lines) < 3 or lines[2].split(",") != list(COLUMNS):
        raise ValueError(f"{path}: not a pathlog")
    descriptions, units = lines[0].split(","), lines[1].split(",")
    rows = [dict(zip(COLUMNS, line.split(","))) for line in lines[3:] if line]
    return descriptions, units, rows


def exists_for(csv_path):
    """True when a pathlog exists beside this poslog."""
    return os.path.isfile(pathlog_path(csv_path))
