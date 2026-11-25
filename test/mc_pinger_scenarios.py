#!/usr/bin/env python3
"""
Pinger modes in master_control, offline (acceptance A4), on the tick-level
harness of mc_harness.py (same simulated clock, plant and in-process path server).

    python3 test/mc_harness.py --pinger-modes        # every scenario, every controller
    python3 test/mc_pinger_scenarios.py               # the same

Scenarios (working tree only; the features do not exist at v1.1):
  pinger_configure    the `configure` accept / refuse table of R1 (refusals:
                      MPC only with follow, replanner only with follow, replanner
                      with has_path false) and the use_pinger alias
  pinger_log_nopath   log mode without a path: ZERO /path_request calls, zero
                      thrust until a manual target, which is then held
  pinger_off_nopath   pinger mode off without a path (has_path false): the same
  pinger_log_path     log mode with a path (MPC allowed): the path is followed,
                      the pinger (fed far away) is never steered to
  pinger_follow_stale follow mode: chases a 'tracking' estimate, HOLDS position
                      when it goes stale (no message for pinger_stale_timeout,
                      or status 'stale'), resumes when it is tracking again; the
                      zero vector before any estimate is never chased
"""

import json
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _estimate(status="tracking", source_status=None):
    d = {"schema": "blueboat_pinger_estimate/1", "status": status, "x": 0.0, "y": 0.0}
    if source_status is not None:
        d["source_status"] = source_status
    return json.dumps(d)


def _feed_pinger(c, goal, status="tracking", source_status=None, zero=False):
    x, y, psi = c.plant.s[:3]
    dx, dy = goal[0] - x, goal[1] - y
    vec = [0.0, 0.0, 0.0] if zero else [math.cos(psi) * dx + math.sin(psi) * dy,
                                        -math.sin(psi) * dx + math.cos(psi) * dy, 0.0]
    c.node.pinger_callback(c.F32(data=vec))
    if status is not None and hasattr(c.node, "pinger_estimate_callback"):
        c.node.pinger_estimate_callback(c.String(data=_estimate(status, source_status)))


def _state(c):
    return json.loads(c.node.run_state_publisher.msgs[-1].data)


class ConfigureTable:
    """R1's configure table; each answer is synchronous (cmd_ok / cmd_detail)."""

    ROWS = [
        # (label, command fields, expect accepted, expected pinger_mode after (or None))
        ("MPC + follow refused", dict(controller_type="MPC", pinger_mode="follow"), False, None),
        ("MPC + use_pinger true (alias of follow) refused", dict(controller_type="MPC", use_pinger=True),
         False, None),
        ("MPC + log accepted", dict(controller_type="MPC", pinger_mode="log"), True, "log"),
        ("MPC + log + replanner accepted", dict(controller_type="MPC", pinger_mode="log",
                                                path_source="replanner"), True, "log"),
        ("log + has_path false + replanner refused", dict(controller_type="MPC", pinger_mode="log",
                                                          has_path=False, path_source="replanner"),
         False, None),
        ("MPC + log + has_path false (mission) accepted", dict(controller_type="MPC", pinger_mode="log",
                                                               has_path=False, path_source="mission"),
         True, "log"),
        ("MPC + off accepted", dict(controller_type="MPC", pinger_mode="off"), True, "off"),
        ("MPC + off + has_path false accepted", dict(controller_type="MPC", pinger_mode="off",
                                                     has_path=False), True, "off"),
        ("off + has_path false + replanner refused", dict(controller_type="MPC", pinger_mode="off",
                                                          has_path=False, path_source="replanner"),
         False, None),
        ("unknown pinger_mode refused", dict(controller_type="MPC", pinger_mode="chase"), False, None),
        ("pinger_mode wins over use_pinger", dict(controller_type="MPC", pinger_mode="log", use_pinger=True),
         True, "log"),
        ("PID + follow + replanner refused", dict(controller_type="PID", pinger_mode="follow",
                                                  path_source="replanner"), False, None),
        ("PID + follow accepted", dict(controller_type="PID", pinger_mode="follow", path_source="mission"),
         True, "follow"),
        ("LoS + use_pinger true accepted as follow", dict(controller_type="LoS", use_pinger=True), True, "follow"),
        ("LoS + use_pinger false accepted as off", dict(controller_type="LoS", use_pinger=False), True, "off"),
        ("LoS + log + pinger_source legacy echoed", dict(controller_type="LoS", pinger_mode="log",
                                                         pinger_source="legacy"), True, "log"),
    ]

    def __init__(self):
        self.i = 0

    def __call__(self, c):
        checks = c.trace.setdefault("checks", [])
        table = c.trace.setdefault("configure_table", [])
        if c.k < 20 or c.k % 5 or self.i >= len(self.ROWS):
            if self.i >= len(self.ROWS) and c.k % 50 == 0:
                st = _state(c)
                for key in ("pinger_mode", "pinger_source", "has_path", "tau", "path_complete"):
                    if not any(n == f"run_state carries {key}" for n, _ in checks):
                        checks.append((f"run_state carries {key}", key in st))
            return
        label, fields, want_ok, want_mode = self.ROWS[self.i]
        self.i += 1
        c.run_command(cmd="configure", note="t", mission="m", **fields)
        st = _state(c)
        ok = bool(c.node.cmd_ok)
        row = {"label": label, "fields": fields, "accepted": ok, "detail": c.node.cmd_detail,
               "pinger_mode": st.get("pinger_mode"), "use_pinger": st.get("use_pinger"),
               "pinger_source": st.get("pinger_source"), "has_path": st.get("has_path")}
        table.append(row)
        good = ok == want_ok and (want_mode is None or st.get("pinger_mode") == want_mode)
        if want_mode is not None:
            good = good and st.get("use_pinger") == (want_mode == "follow")
        if "pinger_source" in fields:
            good = good and st.get("pinger_source") == fields["pinger_source"]
        checks.append((f"configure: {label}", good))


class LogNoPath:
    MODE = "log"

    def __call__(self, c):
        checks = c.trace.setdefault("checks", [])
        ctype = c.node.controller_type
        if c.k == 20:
            c.run_command(cmd="configure", controller_type=ctype, pinger_mode=self.MODE, has_path=False,
                          note="nopath", mission="")
            checks.append((f"configure {self.MODE} + has_path false accepted", bool(c.node.cmd_ok)))
        elif c.k == 40:
            c.run_command(cmd="start")
            checks.append(("run started", c.node.run_active))
        elif c.k == 300:
            c.manual(4.0, 3.0)
        elif c.k == 700:
            c.run_command(cmd="stop")
        _feed_pinger(c, (20.0, -15.0))        # a pinger far away: never steered to


class OffNoPath(LogNoPath):
    """Pinger mode off without a path (2026-11-24, owner): the same as log
    without a path -- no /path_request, Manual Target / arrows only."""
    MODE = "off"


class LogPath:
    def __call__(self, c):
        checks = c.trace.setdefault("checks", [])
        ctype = c.node.controller_type
        if c.k == 20:
            c.run_command(cmd="configure", controller_type=ctype, pinger_mode="log", note="logpath",
                          mission="m")
            checks.append(("configure log (path) accepted for " + ctype, bool(c.node.cmd_ok)))
        elif c.k == 40:
            c.run_command(cmd="start")
        elif c.k == 1100:
            st = _state(c)
            checks.append(("run_state: pinger_mode log, use_pinger false",
                           st.get("pinger_mode") == "log" and st.get("use_pinger") is False))
            c.run_command(cmd="stop")
        _feed_pinger(c, (30.0, 30.0))


class FollowStale:
    """Estimate: none for ticks < 20, tracking 20-399, silent 400-799 (stale by
    age), status 'stale' 800-999, tracking again from 1000."""

    def __call__(self, c):
        k = c.k
        if k < 20:
            status = None
        elif k < 400:
            status = "tracking"
        elif k < 800:
            status = None
        elif k < 1000:
            status = "stale"
        else:
            status = "tracking"
        _feed_pinger(c, (8.0, 5.0), status)


class FollowSource:
    """pinger_source legacy semantics (review #2/#3): the follow law gates on
    the estimate's source_status, never chases a zero vector.
      20-299  filter no_data, source (legacy) tracking  -> chase (the fallback works)
      300-599 source tracking but the vector is zero     -> hold
      600-899 filter tracking, source stale              -> hold
      900-    both tracking                              -> chase"""

    def __call__(self, c):
        k = c.k
        if k < 20:
            return
        if k < 300:
            _feed_pinger(c, (8.0, 5.0), "no_data", "tracking")
        elif k < 600:
            _feed_pinger(c, (8.0, 5.0), "tracking", "tracking", zero=True)
        elif k < 900:
            _feed_pinger(c, (8.0, 5.0), "tracking", "stale")
        else:
            _feed_pinger(c, (8.0, 5.0), "tracking", "tracking")


SCENARIOS = {
    "pinger_configure": dict(session=True, trajectory="circle", ticks=200, controllers=("MPC",),
                             actions={0: [lambda c: c.ready(True)]}, on_tick_factory=ConfigureTable),
    "pinger_log_nopath": dict(session=True, trajectory="circle", ticks=760,
                              actions={0: [lambda c: c.ready(True)]}, on_tick_factory=LogNoPath),
    "pinger_off_nopath": dict(session=True, trajectory="circle", ticks=760,
                              actions={0: [lambda c: c.ready(True)]}, on_tick_factory=OffNoPath),
    "pinger_log_path": dict(session=True, trajectory="circle", ticks=1150,
                            actions={0: [lambda c: c.ready(True)]}, on_tick_factory=LogPath),
    "pinger_follow_stale": dict(use_pinger=True, ticks=1300, controllers=("PID", "LoS"),
                                actions={0: [lambda c: c.ready(True)]}, on_tick_factory=FollowStale),
    "pinger_follow_source": dict(use_pinger=True, ticks=1200, controllers=("PID", "LoS"),
                                 actions={0: [lambda c: c.ready(True)]}, on_tick_factory=FollowSource),
}
ORDER = ("pinger_configure", "pinger_log_nopath", "pinger_off_nopath", "pinger_log_path", "pinger_follow_stale",
         "pinger_follow_source")


def evaluate(name, controller, trace):
    results = list((n, bool(ok)) for n, ok in trace.get("checks", []))
    thrust = np.array(trace["thrust"])
    mon = trace["monitoring"]
    tgt = trace["controller_target"]
    if name in ("pinger_log_nopath", "pinger_off_nopath"):
        results.append(("ZERO /path_request calls without a path",
                        trace["path_requests"] == 0))
        idle = thrust[60:300]
        results.append(("zero thrust with no manual target (ticks 60-299)", np.all(idle == 0.0)))
        results.append(("the manual target is pursued then held (non-zero thrust after it)",
                        np.any(np.abs(thrust[310:700]) > 0.0)))
        results.append(("the pinger is never steered to (no /controller_target)",
                        not any(tgt[k] for k in range(len(tgt)))))
        held = [m for m in mon[600:700] if m is not None]
        results.append(("the monitoring target is the manual target",
                        bool(held) and all(abs(m[4] - 4.0) < 1e-6 and abs(m[5] - 3.0) < 1e-6 for m in held)))
    elif name == "pinger_log_path":
        results.append(("the path is requested", (trace["path_requests"] or 0) > 10))
        results.append(("tau advances along the path", trace["tau"][1050] > 10.0))
        results.append(("the pinger is never steered to (no /controller_target)",
                        not any(tgt[k] for k in range(len(tgt)))))
        far = [m for m in mon[200:1050] if m is not None and math.hypot(m[4] - 30.0, m[5] - 30.0) < 5.0]
        results.append(("the monitored target never goes to the pinger", not far))
    elif name == "pinger_follow_stale":
        chase = [k for k in range(len(tgt)) if tgt[k]]
        results.append(("no chasing before the first estimate (the zero vector is not chased)",
                        not any(k < 20 for k in chase)))
        results.append(("chases while tracking", any(100 <= k < 400 for k in chase)))
        results.append(("holds while the estimate is silent past pinger_stale_timeout",
                        not any(470 <= k < 800 for k in chase)))
        results.append(("holds while the status is 'stale'", not any(800 <= k < 1000 for k in chase)))
        results.append(("resumes when tracking again", any(1000 <= k < 1300 for k in chase)))
        held = [m for m in mon[500:790] if m is not None]
        same = bool(held) and all(abs(m[4] - held[0][4]) < 1e-9 and abs(m[5] - held[0][5]) < 1e-9 for m in held)
        results.append(("the hold target is one latched point", same))
    elif name == "pinger_follow_source":
        chase = [k for k in range(len(tgt)) if tgt[k]]
        results.append(("legacy source tracking while the filter has nothing: chases",
                        any(40 <= k < 300 for k in chase)))
        results.append(("a zero vector is never chased", not any(305 <= k < 600 for k in chase)))
        results.append(("source stale (filter tracking): holds", not any(605 <= k < 900 for k in chase)))
        results.append(("both tracking again: chases", any(905 <= k < 1200 for k in chase)))
    return results


def main(controllers=("PID", "LoS", "MPC")):
    sys.path.insert(0, HERE)
    import mc_harness as h
    total_fail = 0
    for name in ORDER:
        sc = SCENARIOS[name]
        for controller in controllers:
            if controller not in sc.get("controllers", controllers):
                continue
            out = os.path.join(tempfile.mkdtemp(prefix="mc_pinger_"), f"{name}-{controller}.json")
            h._subprocess_run(h.REPO, name, controller, False, out)
            with open(out) as handle:
                trace = json.load(handle)
            results = evaluate(name, controller, trace)
            fails = [n for n, ok in results if not ok]
            total_fail += len(fails)
            print(f"{name:<20} {controller:<4} {len(results) - len(fails)}/{len(results)} checks")
            for n in fails:
                print(f"    FAIL  {n}")
            if name == "pinger_configure":
                for row in trace.get("configure_table", []):
                    print(f"      {'accept' if row['accepted'] else 'REFUSE':<6}  {row['label']:<48} "
                          f"-> mode {row['pinger_mode']}  ({row['detail']})")
    # R10 on a replanner path (from_points, loop False): path_complete false while
    # flying, true once tau passes the plan's end, on path_progress and run_state.
    out = os.path.join(tempfile.mkdtemp(prefix="mc_pinger_"), "replanner-PID.json")
    h._subprocess_run(h.REPO, "replanner", "PID", False, out)
    with open(out) as handle:
        trace = json.load(handle)
    prog = trace["progress"]
    flying = [p for p in prog if p.get("state") == "following"]
    ended = [p for p in prog if p.get("state") == "end_reached"]
    ok = (bool(flying) and not any(p.get("path_complete") for p in flying)
          and bool(ended) and ended[-1].get("path_complete") is True)
    total_fail += 0 if ok else 1
    print(f"{'replanner path_complete':<20} PID  {'1/1' if ok else '0/1'} checks"
          + ("" if ok else "    FAIL  path_complete false while following, true at end_reached"))
    print("pinger modes: " + ("ALL CHECKS PASS" if not total_fail else f"{total_fail} FAILED"))
    return 1 if total_fail else 0


if __name__ == "__main__":
    sys.exit(main())
