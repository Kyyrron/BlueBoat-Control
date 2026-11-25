#!/usr/bin/env python3
r"""
Water Linked UGPS inputs -> `Fix` records for the pinger filter.

ROS-FREE BY CONSTRUCTION (stdlib only). Two sources, one record:

  * `/uw_gps/fix` (std_msgs/String, JSON `blueboat_uwgps_fix/1`, written by
    uwgps_log since 2026-11-24): every field of every polled endpoint, with the
    request/response ROS times. `fixes_from_json`.
  * `/uw_gps_data` (Float32MultiArray, 19 values, the legacy packet): raw xyz at
    [7:10], filtered xyz at [16:19]; no std, no validity, and a failed request is
    all zeros (F1). `fix_from_packet`. Used for old bags and as the fallback.

A Fix is never dropped silently: an unusable one carries `invalid` (the reason)
and the filter counts it.
"""

import json
import math

SCHEMA_UWGPS_FIX = "blueboat_uwgps_fix/1"


class Fix:
    """One Water Linked acoustic fix as received.

    t_rx      receive time on the estimator's clock (s)
    a         (a_x, a_y, a_z) acoustic position as the API returns it (m)
    source    'raw' | 'filtered'
    origin    'uw_gps/fix' | 'uw_gps_data'
    std       WL std (m) or None
    position_valid  bool or None (absent field)
    receivers_valid int or None
    invalid   None, or the reason the fix cannot be used
    key       dedupe key: the same key twice is the same fix
    seq       uwgps_log sequence number (JSON source) or None
    """

    __slots__ = ("t_rx", "a", "source", "origin", "std", "position_valid",
                 "receivers_valid", "invalid", "key", "seq")

    def __init__(self, t_rx, a, source, origin, std=None, position_valid=None,
                 receivers_valid=None, invalid=None, seq=None):
        self.t_rx = float(t_rx)
        self.a = tuple(float(v) for v in a)
        self.source = source
        self.origin = origin
        self.std = None if std is None else float(std)
        self.position_valid = position_valid
        self.receivers_valid = receivers_valid
        self.invalid = invalid
        self.seq = seq
        self.key = (source,) + self.a


def _num(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


MAX_PUBLISH_DELAY_S = 5.0     # a larger cycle delay is not trusted for the correction


def _publish_delay(msg, ep):
    """Seconds between this endpoint's response and the message's publication
    (`t_end`): the later requests of a full cycle (filtered, global, master...)
    delay the raw fix's publication by that much. Both stamps come from
    uwgps_log's own clock, so the difference holds on any receiving clock
    (also in a replay). 0 when unknown or implausible."""
    try:
        d = float(msg.get("t_end")) - float(ep.get("t_resp"))
    except (TypeError, ValueError):
        return 0.0
    return d if (math.isfinite(d) and 0.0 <= d <= MAX_PUBLISH_DELAY_S) else 0.0


def _endpoint_fix(t_rx, ep, source, seq):
    """A Fix from one endpoint record of a /uw_gps/fix message, or None when the
    endpoint was not polled in that cycle."""
    if ep is None:
        return None
    if not isinstance(ep, dict):
        return Fix(t_rx, (0.0, 0.0, 0.0), source, "uw_gps/fix", invalid="malformed_endpoint", seq=seq)
    if not ep.get("ok") or not isinstance(ep.get("data"), dict):
        why = ep.get("error") or f"HTTP {ep.get('status')}"
        return Fix(t_rx, (0.0, 0.0, 0.0), source, "uw_gps/fix", invalid=f"request_failed: {why}", seq=seq)
    d = ep["data"]
    x, y, z = _num(d.get("x")), _num(d.get("y")), _num(d.get("z"))
    if x is None or y is None:
        return Fix(t_rx, (0.0, 0.0, 0.0), source, "uw_gps/fix", invalid="no_xy", seq=seq)
    rv = d.get("receiver_valid")
    n_valid = sum(1 for v in rv if v) if isinstance(rv, (list, tuple)) else None
    pv = d.get("position_valid")
    return Fix(t_rx, (x, y, 0.0 if z is None else z), source, "uw_gps/fix", std=_num(d.get("std")),
               position_valid=None if pv is None else bool(pv), receivers_valid=n_valid, seq=seq)


def parse_fix_message(text):
    """The decoded /uw_gps/fix dict, or None when it is not one."""
    try:
        msg = json.loads(text)
    except ValueError:
        return None
    if not isinstance(msg, dict) or msg.get("schema") != SCHEMA_UWGPS_FIX:
        return None
    return msg


def fixes_from_json(t_rx, msg):
    """{'raw': Fix|None, 'filtered': Fix|None} of one decoded /uw_gps/fix message.

    Each fix's receive time is the message's, minus the time the cycle spent
    on later requests after that endpoint answered (_publish_delay)."""
    eps = msg.get("endpoints")
    if not isinstance(eps, dict):
        eps = {}
    seq = msg.get("seq")
    out = {}
    for name in ("raw", "filtered"):
        ep = eps.get(name)
        delay = _publish_delay(msg, ep) if isinstance(ep, dict) else 0.0
        out[name] = _endpoint_fix(t_rx - delay, ep, name, seq)
    return out


def fix_from_packet(t_rx, data, source):
    """A Fix from the legacy 19-value /uw_gps_data packet (float32 values)."""
    data = [float(v) for v in data]
    if len(data) < 19:
        return Fix(t_rx, (0.0, 0.0, 0.0), source, "uw_gps_data", invalid="short_packet")
    a = data[7:10] if source == "raw" else data[16:19]
    if not all(math.isfinite(v) for v in a):
        return Fix(t_rx, (0.0, 0.0, 0.0), source, "uw_gps_data", invalid="non_finite")
    f = Fix(t_rx, a, source, "uw_gps_data")
    if a[0] == 0.0 and a[1] == 0.0 and a[2] == 0.0:
        f.invalid = "zero_packet"
    return f
