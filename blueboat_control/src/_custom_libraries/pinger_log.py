#!/usr/bin/env python3
"""
The run's pinger per-fix log: `<date>-<note>-pingerlog.csv`, beside the poslog.

ROS-FREE BY CONSTRUCTION (standard library only).

One row per /blueboat/pinger/fix message: every fix the pinger_estimator
processed (duplicates and invalid ones included, with the reason), the boat pose
used at its measurement time, what the filter decided, the estimate after it,
and the legacy pinger at that moment. Two special rows, JSON in `reason` with
commas written as ';':
  * `config` -- the filter parameter set applied from that point;
  * `state`  -- a snapshot of the filter's whole state (the estimator emits one
    at every run start and every few seconds). robot_interface starts a run's
    file AT a snapshot, so the file alone re-runs the live filter bit for bit
    from its first row (pinger_filter_replay.load_pingerlog restores the first
    snapshot; poslog_report's smoothing re-runs it).

Same writing rules as the poslog and the path-update log:
  * row 1: a short description of each column, row 2: its unit, row 3: the
    column names, data from row 4 -- written AT FILE CREATION;
  * no cell contains a comma (free text is sanitised); floats are written with
    repr() so a re-run reads back the exact values;
  * write-once: created with 'x', never overwritten (CM-7). Created at the first
    row, so a run that never saw a fix leaves no file.
Written by robot_interface; filed into the run folder by poslog_report.
"""

import json

from poslog_report import pingerlog_path    # the naming rule, owned by the filer

__all__ = ["COLUMNS", "LEGEND", "PingerLog", "pingerlog_path", "row_from_fix", "config_row",
           "read_rows", "LEGEND_ROWS"]

LEGEND_ROWS = 2

# Column order is part of the record: append only, never reorder (CM-7).
COLUMNS = (
    "t_rx", "t_meas", "fix_seq", "source", "origin", "seq", "a_x", "a_y", "a_z", "std",
    "position_valid", "receivers_valid", "input_invalid", "pose_x", "pose_y", "pose_yaw",
    "pose_yaw_rate", "decision", "reason", "body_x", "body_y", "depth_meas", "world_x", "world_y",
    "R_xx", "R_xy", "R_yy", "nis", "weight", "v_implied", "est_x", "est_y", "est_vx", "est_vy",
    "est_depth", "status", "event", "legacy_x", "legacy_y", "params_hash")

LEGEND = {
    "t_rx": ("Fix receive time", "s (node clock)"),
    "t_meas": ("Fix measurement time", "s (node clock)"),
    "fix_seq": ("Processed fix counter", "-"),
    "source": ("UGPS fix used", "raw/filtered"),
    "origin": ("Input topic", "-"),
    "seq": ("uwgps_log sequence", "-"),
    "a_x": ("Acoustic X as received", "m"),
    "a_y": ("Acoustic Y as received", "m"),
    "a_z": ("Acoustic Z as received", "m"),
    "std": ("WL reported std", "m"),
    "position_valid": ("WL position valid", "0/1"),
    "receivers_valid": ("Valid receivers", "-"),
    "input_invalid": ("Input rejection reason", "-"),
    "pose_x": ("Boat X at t_meas", "m"),
    "pose_y": ("Boat Y at t_meas", "m"),
    "pose_yaw": ("Boat yaw at t_meas", "rad"),
    "pose_yaw_rate": ("Boat yaw rate at t_meas", "rad/s"),
    "decision": ("Filter decision", "-"),
    "reason": ("Decision reason", "-"),
    "body_x": ("Fix body X (mapped)", "m"),
    "body_y": ("Fix body Y (mapped)", "m"),
    "depth_meas": ("Fix depth", "m"),
    "world_x": ("Fix world X", "m"),
    "world_y": ("Fix world Y", "m"),
    "R_xx": ("Fix covariance XX", "m^2"),
    "R_xy": ("Fix covariance XY", "m^2"),
    "R_yy": ("Fix covariance YY", "m^2"),
    "nis": ("Normalised innovation squared", "-"),
    "weight": ("Update weight", "-"),
    "v_implied": ("Implied pinger speed", "m/s"),
    "est_x": ("Estimate X after fix", "m"),
    "est_y": ("Estimate Y after fix", "m"),
    "est_vx": ("Estimate velocity X", "m/s"),
    "est_vy": ("Estimate velocity Y", "m/s"),
    "est_depth": ("Estimate depth", "m"),
    "status": ("Filter status", "-"),
    "event": ("Filter event", "-"),
    "legacy_x": ("Legacy pinger X", "m"),
    "legacy_y": ("Legacy pinger Y", "m"),
    "params_hash": ("Filter parameter hash", "-"),
}


def _clean(text):
    return str(text).replace(",", ";").replace("\r", " ").replace("\n", " ").strip()


def _num(v):
    return "" if v is None else repr(float(v))


def row_from_fix(res):
    """The cells (in COLUMNS order) of one /blueboat/pinger/fix record."""
    pose = res.get("pose") or {}
    post = res.get("posterior") or {}
    R = res.get("R") or [None, None, None]
    body = res.get("body") or [None, None]
    world = res.get("world") or [None, None]
    lw = res.get("legacy_world") or [None, None]
    a = res.get("a") or [None, None, None]
    pv = res.get("position_valid")
    return [_num(res.get("t_rx")), _num(res.get("t_meas")), str(res.get("fix_seq", "")),
            _clean(res.get("source", "")), _clean(res.get("origin", "")),
            "" if res.get("seq") is None else str(res["seq"]),
            _num(a[0]), _num(a[1]), _num(a[2]), _num(res.get("std")),
            "" if pv is None else str(int(bool(pv))),
            "" if res.get("receivers_valid") is None else str(res["receivers_valid"]),
            _clean(res.get("input_invalid") or ""),
            _num(pose.get("x")), _num(pose.get("y")), _num(pose.get("yaw")), _num(pose.get("yaw_rate")),
            _clean(res.get("decision", "")), _clean(res.get("reason", "")),
            _num(body[0]), _num(body[1]), _num(res.get("depth_meas")), _num(world[0]), _num(world[1]),
            _num(R[0]), _num(R[1]), _num(R[2]), _num(res.get("nis")), _num(res.get("weight")),
            _num(res.get("v_implied")), _num(post.get("x")), _num(post.get("y")), _num(post.get("vx")),
            _num(post.get("vy")), _num(post.get("depth")), _clean(res.get("status", "")),
            _clean(res.get("event") or ""), _num(lw[0]), _num(lw[1]), _clean(res.get("params_hash") or "")]


def state_row(rec):
    """A `state` row: the filter snapshot of a /blueboat/pinger/fix `state` record."""
    cells = [""] * len(COLUMNS)
    cells[COLUMNS.index("t_rx")] = _num(rec.get("t_rx"))
    cells[COLUMNS.index("fix_seq")] = str(rec.get("fix_seq", ""))
    cells[COLUMNS.index("source")] = _clean(rec.get("source", ""))
    cells[COLUMNS.index("origin")] = "snapshot"
    cells[COLUMNS.index("decision")] = "state"
    cells[COLUMNS.index("reason")] = json.dumps(rec.get("state"), sort_keys=True).replace(",", ";")
    cells[COLUMNS.index("params_hash")] = _clean(rec.get("params_hash") or "")
    return cells


def config_row(t, params_values, params_hash=""):
    """A `config` row: the parameter set applied from time t on."""
    cells = [""] * len(COLUMNS)
    cells[COLUMNS.index("t_rx")] = _num(t)
    cells[COLUMNS.index("decision")] = "config"
    cells[COLUMNS.index("reason")] = json.dumps(params_values, sort_keys=True).replace(",", ";")
    cells[COLUMNS.index("params_hash")] = _clean(params_hash)
    return cells


def read_rows(path):
    """[{column: cell}] of a per-fix log, with or without the legend rows."""
    import csv
    with open(path, newline="") as fh:
        table = list(csv.reader(fh))
    hi = next(i for i, row in enumerate(table[:LEGEND_ROWS + 1]) if "t_rx" in row)
    header = table[hi]
    return [{name: (row[i] if i < len(row) else "") for i, name in enumerate(header)}
            for row in table[hi + 1:] if row]


class PingerLog:
    """Lazy, write-once writer of one run's per-fix log."""

    def __init__(self, path):
        self.path = path
        self._file = None
        self.rows = 0
        self.closed = False
        self._config = None             # (t, values, hash) applied, written before the first fix

    def set_config(self, t, values, params_hash=""):
        """Record the parameter set in force (written as a `config` row: at file
        creation, and again whenever it changes during the run)."""
        new = (t, dict(values), params_hash)
        if self._config is not None and self._config[2] == params_hash:
            return
        self._config = new
        if self._file is not None:
            self._write(config_row(*new))

    def write_fix(self, res):
        if self.closed:
            return False
        if self._file is None:
            self._open()
            if self._config is not None:
                self._write(config_row(*self._config))
        self._write(state_row(res) if res.get("decision") == "state" else row_from_fix(res))
        return True

    def _open(self):
        self._file = open(self.path, "x", buffering=1)
        self._file.write(",".join(LEGEND[c][0] for c in COLUMNS) + "\n")
        self._file.write(",".join(LEGEND[c][1] for c in COLUMNS) + "\n")
        self._file.write(",".join(COLUMNS) + "\n")
        self._file.flush()

    def _write(self, cells):
        self._file.write(",".join(cells) + "\n")
        self.rows += 1

    def close(self):
        if self._file is not None and not self._file.closed:
            self._file.flush()
            self._file.close()
        self.closed = True
