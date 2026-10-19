#!/usr/bin/env python3
"""
A minimal stand-in for BlueBoat-Replanner, used for the Gazebo tests and as a
worked example of a client of the live path update.

Written ONLY against docs/PATH_UPDATE_INTERFACE.md -- it imports nothing from
Control's code: its own path evaluator implements the interpolation the
interface doc specifies, which is the point (a client never reads Control).

Script, once the run reports `awaiting_path`:
  P0          first path from the published hold pose (east 12 m, left
              half-turn r = 6 m, west 12 m)
  U1          clean update on the straight
  U2-midturn  clean update whose junction lies in the middle of a turn
  U3, U4      back-to-back clean updates (U4 lands before U3's change point)
  BAD-offset  refused (path shifted 1 m) -> checks the path is untouched
  U5-final    clean update; then waits for `end_reached`

    ros2 launch blueboat_control Sim_launch.py controller_type:=PID path_source:=replanner
    python3 test/path_update_demo.py --out /tmp/demo.json
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from blueboat_interfaces.srv import UpdatePath

SPEED = 0.4          # m/s: authored speed (simulation: stay <= 0.45, README)
SAMPLE_DT = 0.5      # s


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OwnPath:
    """The client's own copy of the path it sent -- evaluated with the rule
    of PATH_UPDATE_INTERFACE.md section 3 (linear in t, yaw the short way,
    clamped), and spliced exactly as Control splices it."""

    def __init__(self):
        self.segments = []      # (change_point, t, x, y, yaw)

    def splice(self, c, t, x, y, yaw):
        self.segments = [s for s in self.segments if s[0] < c] + [(c, t, x, y, yaw)]

    def pose(self, tq):
        seg = self.segments[0]
        for s in self.segments:
            if s[0] <= tq:
                seg = s
        _, t, x, y, yaw = seg
        if tq <= t[0]:
            return x[0], y[0], yaw[0]
        if tq >= t[-1]:
            return x[-1], y[-1], yaw[-1]
        i = next(k for k in range(1, len(t)) if t[k] > tq)
        u = (tq - t[i - 1]) / (t[i] - t[i - 1])
        return (x[i - 1] + u * (x[i] - x[i - 1]), y[i - 1] + u * (y[i] - y[i - 1]),
                wrap(yaw[i - 1] + u * wrap(yaw[i] - yaw[i - 1])))


def legs_path(x0, y0, yaw0, t0, legs):
    t, x, y, yaw = [t0], [x0], [y0], [wrap(yaw0)]
    px, py, h, tt = x0, y0, yaw0, t0
    for leg in legs:
        if leg[0] == "S":
            n = max(1, round(leg[1] / (SPEED * SAMPLE_DT)))
            for i in range(1, n + 1):
                s = leg[1] * i / n
                t.append(tt + s / SPEED)
                x.append(px + s * math.cos(h))
                y.append(py + s * math.sin(h))
                yaw.append(wrap(h))
            px, py, tt = px + leg[1] * math.cos(h), py + leg[1] * math.sin(h), tt + leg[1] / SPEED
        else:
            sgn = 1.0 if leg[0] == "L" else -1.0
            r, ang = leg[1], leg[2]
            cx, cy = px - sgn * r * math.sin(h), py + sgn * r * math.cos(h)
            n = max(1, round(r * ang / (SPEED * SAMPLE_DT)))
            for i in range(1, n + 1):
                a = ang * i / n
                hh = h + sgn * a
                t.append(tt + r * a / SPEED)
                x.append(cx + sgn * r * math.sin(hh))
                y.append(cy - sgn * r * math.cos(hh))
                yaw.append(wrap(hh))
            h = h + sgn * ang
            px, py, tt = cx + sgn * r * math.sin(h), cy - sgn * r * math.cos(h), tt + r * ang / SPEED
    return t, x, y, yaw


class Demo(Node):
    def __init__(self):
        super().__init__("path_update_demo")
        self.client = self.create_client(UpdatePath, "/blueboat/update_path")
        self.progress = None
        self.events = []
        self.create_subscription(String, "/blueboat/path_progress", self._on_progress, 10)
        self.create_subscription(String, "/blueboat/path_update_event", self._on_event,
                                 QoSProfile(depth=50))
        self.own = OwnPath()
        self.answers = []
        self.checks = []

    def _on_progress(self, msg):
        self.progress = json.loads(msg.data)

    def _on_event(self, msg):
        self.events.append(json.loads(msg.data))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, predicate, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.progress is not None and predicate(self.progress):
                return True
        raise TimeoutError(f"timed out waiting for {what} (last progress: {self.progress})")

    def send(self, rid, t, x, y, yaw, change_point=float("nan")):
        req = UpdatePath.Request()
        req.id, req.change_point, req.frame = rid, float(change_point), "world"
        req.t, req.x, req.y, req.yaw = list(t), list(x), list(y), list(yaw)
        future = self.client.call_async(req)
        sent = time.monotonic()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - sent > 10.0:
                raise TimeoutError(f"no answer to {rid}")
        resp = future.result()
        self.answers.append({"id": rid, "accepted": resp.accepted, "reason": resp.reason,
                             "message": resp.message, "active_id": resp.active_id,
                             "tau": resp.tau, "change_point": float(change_point),
                             "round_trip_ms": (time.monotonic() - sent) * 1e3})
        self.get_logger().info(f"{rid}: {resp.reason} - {resp.message}")
        if resp.accepted:
            c = t[0] if math.isnan(change_point) else change_point
            self.own.splice(c, t, x, y, yaw)
        return resp

    def continue_from(self, c, legs):
        x, y, yaw = self.own.pose(c)
        return legs_path(x, y, yaw, c, legs)

    def next_c(self, margin):
        """A change point the interface doc's rule accepts now: min_change_point
        plus the client's own latency margin."""
        rclpy.spin_once(self, timeout_sec=0.05)
        return self.progress["min_change_point"] + margin

    def run(self, timeout):
        start = time.monotonic()
        if not self.client.wait_for_service(timeout_sec=60.0):
            raise TimeoutError("/blueboat/update_path not available")
        self.wait(lambda p: p["state"] == "awaiting_path" and p["hold"] is not None, 120,
                  "awaiting_path with a hold pose")
        self.spin_for(3.0)
        hold = self.progress["hold"]
        t, x, y, yaw = legs_path(hold["x"], hold["y"], hold["yaw"], 0.0,
                                 [("S", 12.0), ("L", 6.0, math.pi), ("S", 12.0)])
        self.checks.append(["P0 accepted", self.send("P0", t, x, y, yaw).accepted])

        self.wait(lambda p: p["tau"] >= 6.0, 120, "tau >= 6")
        c1 = self.next_c(1.5)
        self.checks.append(["U1 accepted", self.send(
            "U1", *self.continue_from(c1, [("S", 8.0), ("L", 4.0, math.pi / 2), ("S", 14.0)]),
            change_point=c1).accepted])

        turn_mid = c1 + 8.0 / SPEED + 0.5 * 4.0 * (math.pi / 2) / SPEED
        self.wait(lambda p: p["min_change_point"] is not None
                  and p["min_change_point"] >= turn_mid - 1.0, 240, "U1's turn approaching")
        self.checks.append(["U2-midturn accepted (junction inside U1's turn)", self.send(
            "U2-midturn", *self.continue_from(turn_mid, [("S", 6.0), ("R", 5.0, math.pi / 2),
                                                          ("S", 12.0)]),
            change_point=turn_mid).accepted])

        self.wait(lambda p: p["tau"] >= turn_mid + 4.0, 240, "past U2")
        c3 = self.next_c(6.0)
        a3 = self.send("U3", *self.continue_from(c3, [("S", 10.0), ("L", 5.0, math.pi / 3),
                                                      ("S", 10.0)]), change_point=c3)
        c4 = self.next_c(2.0)
        a4 = self.send("U4", *self.continue_from(c4, [("S", 8.0), ("R", 6.0, math.pi / 4),
                                                      ("S", 16.0)]), change_point=c4)
        self.checks.append(["U3 and U4 accepted back-to-back", a3.accepted and a4.accepted])
        self.checks.append(["U4 lands before U3's change point", c4 < c3])

        self.wait(lambda p: p["tau"] >= c4 + 3.0, 240, "past U4")
        before = {k: self.progress[k] for k in ("active_id", "pending_ids", "t_end")}
        cb = self.next_c(2.0)
        t, x, y, yaw = self.continue_from(cb, [("S", 10.0)])
        bad = self.send("BAD-offset", t, x, [v + 1.0 for v in y], yaw, change_point=cb)
        self.checks.append(["BAD-offset refused discontinuous_position",
                            bad.reason == "discontinuous_position"])
        self.spin_for(0.5)
        after = {k: self.progress[k] for k in ("active_id", "pending_ids", "t_end")}
        self.checks.append(["refusal left the path untouched", before == after])

        self.spin_for(3.0)
        c5 = self.next_c(1.0)
        self.checks.append(["U5-final accepted", self.send(
            "U5-final", *self.continue_from(c5, [("S", 6.0)]), change_point=c5).accepted])

        remaining = max(30.0, timeout - (time.monotonic() - start))
        self.wait(lambda p: p["state"] == "end_reached", remaining, "end_reached")
        self.spin_for(5.0)
        activated = [e["id"] for e in self.events if e["event"] == "activated"]
        self.checks.append(["activations P0 U1 U2-midturn U4 U5-final",
                            activated == ["P0", "U1", "U2-midturn", "U4", "U5-final"]])
        self.checks.append(["end_reached once",
                            sum(e["event"] == "end_reached" for e in self.events) == 1])
        return {"answers": self.answers, "events": self.events, "checks": self.checks,
                "final_progress": self.progress,
                "sent": [{"c": s[0], "t": s[1], "x": s[2], "y": s[3], "yaw": s[4]}
                         for s in self.own.segments]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=900.0)
    args = ap.parse_args()
    rclpy.init()
    node = Demo()
    try:
        result = node.run(args.timeout)
        ok = all(passed for _, passed in result["checks"])
    except Exception as exc:                            # noqa: BLE001 - reported
        result = {"error": f"{type(exc).__name__}: {exc}", "answers": node.answers,
                  "events": node.events, "checks": node.checks,
                  "final_progress": node.progress}
        ok = False
    with open(args.out, "w") as handle:
        json.dump(result, handle, indent=1)
    for name, passed in result["checks"]:
        print(f"{'PASS' if passed else 'FAIL'}  {name}")
    if "error" in result:
        print("ERROR", result["error"])
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
