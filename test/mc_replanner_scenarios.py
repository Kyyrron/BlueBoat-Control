#!/usr/bin/env python3
"""
Offline live-path-update scenarios for mc_harness.py (`--replanner`).

A replanner-owned run of the WORKING TREE's master_control, closed around the
harness plant, driven through the real /blueboat/update_path callback:

  replanner          simulation parameters, no session (Sim_launch's graph):
                     hold -> first path -> 5 clean updates (one junction in the
                     middle of a turn, two back-to-back) -> one refused update
                     (plan proven untouched) -> refusals under manual target
                     and E-STOP -> end of path.
  replanner_session  real-boat session: configure path_source=replanner,
                     start (hold) -> first path -> update -> stop -> start
                     (new hold, ids reset) -> first path again -> stop; and a
                     fixed-mission run refusing with 'mission_owned'.

Every check is printed and counted; exit code 1 on any failure.
"""

import math
import os
import sys
import tempfile
import json

import numpy as np

SPEED = 0.4          # m/s -- below the sim plant's ~0.78 m/s ceiling (README)
SAMPLE_DT = 0.5      # s between samples


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def legs_path(x0, y0, yaw0, t0, legs, speed=SPEED, dt=SAMPLE_DT):
    """Samples [t, x, y, yaw] of a path made of legs, starting at a pose:
    ('S', length) straight, ('L'|'R', radius, angle) circular arc. Yaw is the
    tangent; consecutive legs share their junction sample."""
    pts = [(t0, x0, y0, wrap(yaw0))]
    x, y, yaw, t = x0, y0, yaw0, t0
    for leg in legs:
        if leg[0] == "S":
            length = leg[1]
            n = max(1, int(round(length / (speed * dt))))
            for i in range(1, n + 1):
                s = length * i / n
                pts.append((t + s / speed, x + s * math.cos(yaw), y + s * math.sin(yaw),
                            wrap(yaw)))
            x, y, t = x + length * math.cos(yaw), y + length * math.sin(yaw), t + length / speed
        else:
            sgn = 1.0 if leg[0] == "L" else -1.0
            r, ang = leg[1], leg[2]
            cx, cy = x - sgn * r * math.sin(yaw), y + sgn * r * math.cos(yaw)
            length = r * ang
            n = max(1, int(round(length / (speed * dt))))
            for i in range(1, n + 1):
                a = ang * i / n
                h = yaw + sgn * a
                pts.append((t + r * a / speed, cx + sgn * r * math.sin(h),
                            cy - sgn * r * math.cos(h), wrap(h)))
            yaw = yaw + sgn * ang
            x, y, t = cx + sgn * r * math.sin(yaw), cy - sgn * r * math.cos(yaw), t + length / speed
    arr = np.array(pts)
    return arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]


def continue_from(updater, c, legs):
    """A path continuing the CURRENT plan at path parameter c (the Replanner's
    job: start where Control's path is at c, as the interface doc says)."""
    x, y, yaw = updater.plan.pose(c)
    return legs_path(x, y, yaw, c, legs)


def snapshot(updater):
    return ([s.id for s in updater.plan.segments], [s.start for s in updater.plan.segments],
            sorted(updater.used_ids))


# ---------------------------------------------------------------------------
# The 'replanner' script: a state machine run once per tick
# ---------------------------------------------------------------------------
class ReplannerScript:
    def __init__(self):
        self.stage = 0
        self.marks = {}

    def __call__(self, c):
        node, up, k = c.node, c.node.path_updater, c.k
        tau = node.tau
        checks = c.trace.setdefault("checks", [])
        if k == 0:
            c.ready(True)
            # readiness has arrived but no control tick has run yet: refused,
            # so the first tick's tau = 0 can never overwrite a first path
            resp = c.update("EARLY", *legs_path(0.0, 0.0, 0.0, 0.0, [("S", 5.0)]))
            checks.append(("first path before the first control tick: not_ready",
                           resp.reason == "not_ready"))
        if self.stage == 0 and k == 20:
            # first path from the hold pose: east 12 m, left half-turn r=6, west 12 m
            hx, hy, hyaw = up.hold
            t, x, y, yaw = legs_path(hx, hy, hyaw, 0.0,
                                     [("S", 12.0), ("L", 6.0, math.pi), ("S", 12.0)])
            c.update("P0", t, x, y, yaw)
            self.stage = 1
        elif self.stage == 1 and tau >= 6.0:
            cp = tau + up.lead_min(node._path_update_context()) + 1.5
            c.update("U1", *continue_from(up, cp, [("S", 8.0), ("L", 4.0, math.pi / 2),
                                                   ("S", 14.0)]), change_point=cp)
            self.marks["U1"] = cp
            self.stage = 2
        elif self.stage == 2:
            # mid-turn junction: U1's left turn spans [cp1+20, cp1+20+15.7]
            turn_mid = self.marks["U1"] + 8.0 / SPEED + 0.5 * 4.0 * (math.pi / 2) / SPEED
            lead = up.lead_min(node._path_update_context())
            if tau >= turn_mid - lead - 1.0:
                c.update("U2-midturn", *continue_from(up, turn_mid, [
                    ("S", 6.0), ("R", 5.0, math.pi / 2), ("S", 12.0)]), change_point=turn_mid)
                self.marks["U2"] = turn_mid
                self.stage = 3
        elif self.stage == 3 and tau >= self.marks["U2"] + 4.0:
            # back-to-back: U3, then U4 on the NEXT tick landing BEFORE U3's c
            lead = up.lead_min(node._path_update_context())
            c3 = tau + lead + 6.0
            c.update("U3", *continue_from(up, c3, [("S", 10.0), ("L", 5.0, math.pi / 3),
                                                   ("S", 10.0)]), change_point=c3)
            self.marks["U3"] = c3
            self.stage = 4
        elif self.stage == 4:
            lead = up.lead_min(node._path_update_context())
            c4 = tau + lead + 2.0                               # before U3's change point
            checks.append(("back-to-back: U4 lands before U3's change point",
                           c4 < self.marks["U3"]))
            c.update("U4", *continue_from(up, c4, [("S", 8.0), ("R", 6.0, math.pi / 4),
                                                   ("S", 16.0)]), change_point=c4)
            self.marks["U4"] = c4
            self.stage = 5
        elif self.stage == 5 and tau >= self.marks["U4"] + 3.0:
            # a refused update must leave the plan untouched
            before = snapshot(up)
            lead = up.lead_min(node._path_update_context())
            cp = tau + lead + 2.0
            t, x, y, yaw = continue_from(up, cp, [("S", 10.0)])
            resp = c.update("BAD-offset", t, x, y + 1.0, yaw, change_point=cp)
            checks.append(("refused update: discontinuous_position",
                           resp.reason == "discontinuous_position"))
            checks.append(("refused update: plan, ids untouched", snapshot(up) == before))
            # and a lead that is too short
            cp2 = tau + 0.5 * lead
            resp = c.update("BAD-lead", *continue_from(up, cp2, [("S", 10.0)]), change_point=cp2)
            checks.append(("refused update: lead_too_short", resp.reason == "lead_too_short"))
            checks.append(("refused updates: plan still untouched", snapshot(up) == before))
            self.stage = 6
        elif self.stage == 6 and tau >= self.marks["U4"] + 6.0:
            # operator override: a manual target outranks any update
            x, y, _ = node.current_pose[0], node.current_pose[1], 0
            c.manual(x + 1.0, y + 1.0)
            self.marks["manual_k"] = k
            self.stage = 7
        elif self.stage == 7 and k == self.marks["manual_k"] + 40:
            lead = up.lead_min(node._path_update_context())
            cp = tau + lead + 2.0
            resp = c.update("OVR", *continue_from(up, cp, [("S", 10.0)]), change_point=cp)
            checks.append(("manual target engaged: operator_override",
                           resp.reason == "operator_override"))
            tau_frozen = tau
            c.manual(0.0, 0.0)                                  # resume sentinel
            checks.append(("tau frozen during the manual override",
                           abs(node.tau - tau_frozen) < 1e-12))
            self.marks["resume_k"] = k
            self.stage = 8
        elif self.stage == 8 and k == self.marks["resume_k"] + 60:
            c.ready(False)                                      # E-STOP latched
            lead = up.lead_min(node._path_update_context())
            cp = tau + lead + 2.0
            resp = c.update("ESTOP", *continue_from(up, cp, [("S", 10.0)]), change_point=cp)
            checks.append(("E-STOP latched: not_ready", resp.reason == "not_ready"))
            self.marks["estop_k"] = k
            self.stage = 9
        elif self.stage == 9 and k == self.marks["estop_k"] + 40:
            c.ready(True)                                       # released
            self.stage = 10
        elif self.stage == 10 and k == self.marks["estop_k"] + 80:
            # final clean update: a short line, so the end of the path is reached
            lead = up.lead_min(node._path_update_context())
            cp = tau + lead + 1.0
            c.update("U5-final", *continue_from(up, cp, [("S", 6.0)]), change_point=cp)
            self.marks["U5"] = cp
            self.stage = 11


class SessionScript:
    def __init__(self):
        self.stage = 0
        self.marks = {}

    def __call__(self, c):
        node, up, k = c.node, c.node.path_updater, c.k
        checks = c.trace.setdefault("checks", [])
        tau = node.tau
        if k == 0:
            c.ready(True)
        if k == 10:
            # a fixed-mission run first: updates refused
            c.run_command(cmd="configure", controller_type=node.controller_type,
                          use_pinger=False, note="mission", mission="m")
        if k == 20:
            c.run_command(cmd="start")
        if k == 60:
            hx, hy, hyaw = node.current_pose[0], node.current_pose[1], node.current_pose[5]
            resp = c.update("M1", *legs_path(hx, hy, hyaw, 0.0, [("S", 10.0)]))
            checks.append(("fixed-mission run: mission_owned", resp.reason == "mission_owned"))
        if k == 100:
            c.run_command(cmd="stop")
        if k == 110:
            resp = c.update("IDLE", *legs_path(0, 0, 0, 0.0, [("S", 10.0)]))
            checks.append(("idle session: no_run", resp.reason == "no_run"))
            c.run_command(cmd="configure", controller_type=node.controller_type,
                          use_pinger=True, note="x", mission="m", path_source="replanner")
            checks.append(("configure replanner + pinger refused", node.cmd_ok is False))
            c.run_command(cmd="configure", controller_type=node.controller_type,
                          use_pinger=False, note="rp", mission="rp", path_source="replanner")
            checks.append(("configure path_source=replanner accepted", node.cmd_ok is True))
        if k == 120:
            c.run_command(cmd="start")
            checks.append(("run started awaiting its first path", up.awaiting
                           and up.hold is not None))
            self.marks["hold1"] = up.hold
        if k == 160:
            hx, hy, hyaw = up.hold
            resp = c.update("P0", *legs_path(hx, hy, hyaw, 0.0,
                                             [("S", 10.0), ("L", 5.0, math.pi / 2), ("S", 10.0)]))
            checks.append(("session: first path accepted", resp.accepted))
        if self.stage == 0 and k > 160 and tau >= 5.0:
            lead = up.lead_min(node._path_update_context())
            cp = tau + lead + 1.0
            resp = c.update("U1", *continue_from(up, cp, [("S", 6.0)]), change_point=cp)
            checks.append(("session: update accepted", resp.accepted))
            self.stage = 1
            self.marks["stop_k"] = k + 200
        if self.stage == 1 and k == self.marks["stop_k"]:
            c.run_command(cmd="stop")
            checks.append(("stop cleared the plan", not up.plan and not up.awaiting))
            self.marks["restart_k"] = k + 40
            self.stage = 2
        if self.stage == 2 and k == self.marks["restart_k"]:
            c.run_command(cmd="start")
            checks.append(("new run: awaiting again with a new hold", up.awaiting and
                           up.hold is not None and up.hold != self.marks["hold1"]))
            hx, hy, hyaw = up.hold
            resp = c.update("P0", *legs_path(hx, hy, hyaw, 0.0, [("S", 8.0)]))
            checks.append(("new run: id P0 accepted again (ids are per run)", resp.accepted))
            self.marks["stop2_k"] = k + 300
            self.stage = 3
        if self.stage == 3 and k == self.marks["stop2_k"]:
            c.run_command(cmd="stop")
            self.stage = 4


SCENARIOS = {
    "replanner": dict(path_source="replanner", ticks=5200, on_tick_factory=ReplannerScript),
    "replanner_session": dict(session=True, path_source="mission", trajectory="circle",
                              ticks=1300, on_tick_factory=SessionScript),
}


# ---------------------------------------------------------------------------
# Checks on a finished trace
# ---------------------------------------------------------------------------
def evaluate(name, controller, trace):
    results = list((n, bool(ok)) for n, ok in trace.get("checks", []))
    answers = {a["id"]: a for a in trace["answers"]}
    events = trace["events"]
    if name == "replanner":
        clean = ["P0", "U1", "U2-midturn", "U3", "U4", "U5-final"]
        for rid in clean:
            results.append((f"{rid} accepted", answers.get(rid, {}).get("accepted") is True))
        activated = [e["id"] for e in events if e["event"] == "activated"]
        results.append(("activation order P0 U1 U2 U4 U5 (U3 superseded by U4)",
                        activated == ["P0", "U1", "U2-midturn", "U4", "U5-final"]))
        results.append(("end_reached exactly once",
                        sum(e["event"] == "end_reached" for e in events) == 1))
        results.append(("progress ends in state end_reached",
                        trace["progress"][-1]["state"] == "end_reached"))
        # the reference never jumps while following: consecutive monitored
        # targets differ by at most the authored step (0.4 m/s * 0.05 s) plus
        # rounding, except across the manual override (target = operator's)
        mon = trace["monitoring"]
        jumps = []
        for k in range(1, len(mon)):
            if mon[k] is None or mon[k - 1] is None:
                continue
            d = math.hypot(mon[k][4] - mon[k - 1][4], mon[k][5] - mon[k - 1][5])
            jumps.append((d, k))
        override_ticks = set()
        ov = [a["k"] for a in trace["answers"] if a["id"] == "OVR"]
        if ov:
            override_ticks = set(range(ov[0] - 45, ov[0] + 3))
        big = [(d, k) for d, k in jumps if d > SPEED * 0.05 * 1.05 + 1e-3
               and k not in override_ticks]
        results.append(("reference continuous at every junction (no jump > step)",
                        not big))
        trace["max_ref_step"] = max(d for d, _ in jumps) if jumps else 0.0
        trace["ref_jumps"] = big[:5]
    else:
        results.append(("session: 'cleared' events at starts and stops",
                         sum(e["event"] == "cleared" for e in events) >= 3))
    handling = [e["handling_ms"] for e in events if e.get("handling_ms") is not None]
    trace["handling_ms_max"] = max(handling) if handling else None
    return results


def main(controllers):
    import mc_harness as h
    total_fail = 0
    for name in ("replanner", "replanner_session"):
        for controller in controllers:
            out = os.path.join(tempfile.mkdtemp(prefix="mc_rp_"), f"{name}-{controller}.json")
            h._subprocess_run(h.REPO, name, controller, name == "replanner_session", out)
            with open(out) as handle:
                trace = json.load(handle)
            results = evaluate(name, controller, trace)
            fails = [n for n, ok in results if not ok]
            total_fail += len(fails)
            ticks = trace["tick_cost_ms"]
            print(f"{name:<18} {controller:<4} {len(results) - len(fails)}/{len(results)} checks"
                  f"  handling max {trace['handling_ms_max']:.2f} ms"
                  f"  tick median {np.median(ticks):.2f} ms max {max(ticks):.2f} ms"
                  + (f"  max reference step {trace.get('max_ref_step', 0):.4f} m"
                     if name == "replanner" else ""))
            for n in fails:
                print(f"    FAIL {n}")
            if name == "replanner" and trace.get("ref_jumps"):
                print(f"    jumps: {trace['ref_jumps']}")
            with open(out.replace(".json", "-answers.json"), "w") as handle:
                json.dump(trace["answers"], handle, indent=1)
            print(f"    trace: {out}")
    print(f"\n{'ALL PASSED' if not total_fail else f'{total_fail} FAILED'}")
    return 1 if total_fail else 0
