#!/usr/bin/env python3
r"""
The whole pinger pipeline as an event-driven, ROS-free object: the pinger_estimator
node calls it from its callbacks, the pinger_filter_replay CLI calls it from a
bag, so online and offline run the SAME code on the same event sequence.

ROS-FREE BY CONSTRUCTION (numpy + the pinger_* modules).

Events (t = receive time on the caller's clock, seconds):
  on_odom(t, x, y, yaw, vx, vy, vz)  /blueboat/odom (local ENU pose, body twist)
  on_imu(t, wz)                      /mavros/imu/data yaw rate (legacy dead reckoning)
  on_packet(t, data19)               /uw_gps_data (legacy reseed; filter input only
                                     when no /uw_gps/fix arrived for JSON_SILENT_S)
  on_fix_json(t, text)               /uw_gps/fix (the filter's input)
Outputs:
  on_odom -> {'body': [3] or None, 'legacy_body': [3] or None, 'filtered_body': [3] or None}
             'body' is what goes on /blueboat/pinger_coordinates for the chosen
             pinger_source ('filter': zeros until initialised; 'legacy': exactly
             robot_interface v1.1's vector, None on the first odom as before).
  on_packet / on_fix_json -> list of fix records (/blueboat/pinger/fix payloads)
  estimate(t) -> /blueboat/pinger/estimate payload; `status` is the FILTER's,
             `source_status` that of the source /blueboat/pinger_coordinates
             carries (the filter's, or with 'legacy' the legacy input's age):
             master_control gates following on `source_status`.
  snapshot_record(t, reason) -> a `state` record for /blueboat/pinger/fix: the
             filter's whole state, so a per-fix log may start mid-session and
             still re-run bit for bit (pinger_filter_replay restores it).
"""

import math

import pinger_input as pin
from legacy_pinger import LegacyPinger
from pinger_filter import STATUS_CODE, PingerFilter, PoseBuffer, _f

SCHEMA_ESTIMATE = "blueboat_pinger_estimate/1"
SCHEMA_FIX = "blueboat_pinger_fix/1"
SCHEMA_CONFIG = "blueboat_pinger_filter_config/1"
SOURCES = ("filter", "legacy")
JSON_SILENT_S = 3.0
ZERO_BODY = [0.0, 0.0, 0.0]


def legacy_world(x, y, yaw, body):
    """[x, y] world point of a legacy body vector at the boat pose (x, y, yaw),
    or None while there is none (None, or all zero = not yet detected). One
    formula for the per-fix log's legacy_x/legacy_y and the replay's legacy track."""
    if body is None or (body[0] == 0.0 and body[1] == 0.0 and body[2] == 0.0):
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    return [x + c * body[0] - s * body[1], y + s * body[0] + c * body[1]]


class EstimatorCore:

    def __init__(self, params, pinger_source="filter", keep_steps=True):
        if pinger_source not in SOURCES:
            raise ValueError(f"pinger_source {pinger_source!r} is not one of {', '.join(SOURCES)}")
        self.pinger_source = pinger_source
        self.keep_steps = keep_steps
        self.last_legacy_packet_t = None    # receive time of the last usable legacy packet
        self.legacy = LegacyPinger()
        self.last_pose = None
        self.last_legacy_body = None
        self.last_json_t = None
        self.fix_seq = 0
        self.set_params(params)

    def set_params(self, params):
        """Apply a parameter set. The filter restarts (a new latency or mapping
        invalidates the old state); legacy and the pose history are kept."""
        self.params = params            # a pinger_params.Params
        self.values = dict(params.values)
        self.filter = PingerFilter(self.values, keep_steps=self.keep_steps)
        old = getattr(self, "poses", None)
        self.poses = PoseBuffer(self.values["pose_buffer_s"])
        if old is not None:
            for t, x, y, yaw in zip(old.t, old.x, old.y, old.yaw):
                self.poses.add(t, x, y, yaw)

    # ---------------------------------------------------------------- inputs
    def on_odom(self, t, x, y, yaw, vx, vy, vz):
        self.poses.add(t, x, y, yaw)
        self.last_pose = (t, x, y, yaw)
        leg = self.legacy.on_odom(t, (vx, vy, vz), x, y, yaw)
        if leg is not None:
            self.last_legacy_body = leg
        filt = self.filter.body_at(x, y, yaw)
        legacy_body = None if leg is None else [float(v) for v in leg]
        filtered_body = None if filt is None else [float(v) for v in filt]
        # `body` (/blueboat/pinger_coordinates) is one of the two, as its own list.
        if self.pinger_source == "legacy":
            body = None if legacy_body is None else list(legacy_body)
        else:
            body = list(ZERO_BODY) if filtered_body is None else list(filtered_body)
        return {"body": body, "legacy_body": legacy_body, "filtered_body": filtered_body}

    def on_imu(self, t, wz):
        self.legacy.on_imu(wz)

    def json_active(self, t):
        return self.last_json_t is not None and t - self.last_json_t <= JSON_SILENT_S

    def on_packet(self, t, data):
        f = [float(v) for v in data[16:19]] if len(data) >= 19 else []
        if len(f) == 3 and all(math.isfinite(v) for v in f):
            # A non-finite packet (never seen from uwgps_log) would turn the
            # legacy vector -- possibly the follow target -- into NaN: skipped.
            self.legacy.on_ugps(data)
            if any(v != 0.0 for v in f):
                self.last_legacy_packet_t = t
        if self.json_active(t) or not self.values["use_legacy_fallback_input"]:
            return []
        return [self._process(pin.fix_from_packet(t, data, self.values["source"]))]

    def on_fix_json(self, t, text):
        msg = pin.parse_fix_message(text)
        if msg is None:
            return []
        self.last_json_t = t
        fix = pin.fixes_from_json(t, msg)[self.values["source"]]
        if fix is None:
            return []
        return [self._process(fix)]

    def process_with_pose(self, fix, pose):
        """Offline re-run from a per-fix log: the fix with the pose recorded for it."""
        return self._finish(self.filter.process(fix, pose))

    # --------------------------------------------------------------- outputs
    def _process(self, fix):
        t_m = fix.t_rx - self.values["latency_s"]
        pose = self.poses.at(t_m, self.values["max_pose_extrap_s"], self.values["max_pose_gap_s"])
        return self._finish(self.filter.process(fix, pose))

    def _finish(self, res):
        self.fix_seq += 1
        res["schema"] = SCHEMA_FIX
        res["fix_seq"] = self.fix_seq
        res["params_hash"] = self.params.hash
        lw = None
        if self.last_pose is not None:
            _, x, y, yaw = self.last_pose
            lw = legacy_world(x, y, yaw, self.last_legacy_body)
            if lw is not None and not all(math.isfinite(v) for v in lw):
                lw = None
        res["legacy_world"] = lw
        return res

    def legacy_status(self, t):
        """The legacy input's freshness, on the filter's coast / stale times."""
        if self.last_legacy_packet_t is None:
            return "no_data"
        age = t - self.last_legacy_packet_t
        if age <= self.values["coast_after_s"]:
            return "tracking"
        return "coasting" if age <= self.values["stale_timeout_s"] else "stale"

    def snapshot_record(self, t, reason, run=None):
        """A `state` record (/blueboat/pinger/fix): the filter's whole state now."""
        self.fix_seq += 1
        return {"schema": SCHEMA_FIX, "decision": "state", "reason": str(reason), "t_rx": float(t),
                "run": run, "fix_seq": self.fix_seq, "params_hash": self.params.hash,
                "source": self.values["source"], "origin": "snapshot", "state": self.filter.snapshot()}

    def estimate(self, t):
        out = self.filter.estimate(t)
        out["filter_status"] = out["status"]
        src = out["status"] if self.pinger_source == "filter" else self.legacy_status(t)
        out["source_status"] = src
        out["source_status_code"] = STATUS_CODE[src]
        out["schema"] = SCHEMA_ESTIMATE
        out["source"] = self.pinger_source
        out["input"] = self.values["source"]
        out["input_topic"] = "/uw_gps/fix" if self.json_active(t) else "/uw_gps_data"
        out["params_hash"] = self.params.hash
        if self.last_pose is not None and out["x"] is not None:
            _, x, y, _yaw = self.last_pose
            out["range_m"] = _f(math.hypot(out["x"] - x, out["y"] - y))
        else:
            out["range_m"] = None
        return out

    def config_payload(self, seq=None, ok=True, detail="", preset=None):
        d = self.params.as_dict()
        if preset is not None:
            d["preset"] = preset
        d.update({"schema": SCHEMA_CONFIG, "pinger_source": self.pinger_source, "seq": seq,
                  "ok": bool(ok), "detail": str(detail),
                  "mapping": {"y_sign": self.values["y_sign"], "z_sign": self.values["z_sign"],
                              "mount_yaw_deg": self.values["mount_yaw_deg"],
                              "lever_x_m": self.values["lever_x_m"], "lever_y_m": self.values["lever_y_m"],
                              "latency_s": self.values["latency_s"]}})
        return d
