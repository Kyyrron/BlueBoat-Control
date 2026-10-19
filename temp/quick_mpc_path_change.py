#!/usr/bin/env python3
"""
TEMPORARY quick test of the live path update -- one simulation, one path change.

1. Starts the simulation the way the Mission Control Station does for an empty
   world (Sim_launch.py robot_file / trajectory / controller_type / note), plus
   `path_source:=replanner` -- which is exactly what typing
   `path_source:=replanner` in MCS's Launch dialog "Extra args" field adds.
2. Waits until Control reports `awaiting_path` (boat holding at its spawn pose).
3. PATH A (first path): 30 m straight ahead at 0.4 m/s. The boat follows it ~10 s.
4. PATH C (INCOMPATIBLE): starts 1.5 m to the side of path A at the change
   point, turned 30 deg  -> must be REFUSED, and path A must keep being followed.
5. PATH B (COMPATIBLE): continues path A from the change point, turns left 90 deg
   (radius 5 m) and runs 10 m -> must be ACCEPTED and then followed.
6. Watches the boat for 30 s after B becomes active, stops the simulation (the
   run is filed like any other, with its -pathlog.csv), prints PASS / FAIL.

Run (from BlueBoat-Control/). The venv is REQUIRED: master_control and
simulation_interface import casadi / acados_template, which only the venv has
(the script checks this first and stops with instructions if it is missing):
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    source ~/ros2_ws/.venv/bin/activate
    python3 temp/quick_mpc_path_change.py            # headless, MPC
    python3 temp/quick_mpc_path_change.py --gui      # Gazebo + RViz windows

With --gui, RViz opens with temp/quick_path_change.rviz: Control's usual view
plus the paths this script sent -- green = the path Control now follows (A,
then A up to the change point + B), red = the refused path C. (Control's own
/set_path display shows the fixed-mission path, which in a replanner run is a
single point at the origin; the moving arrow is the controller's target.)

Written against docs/PATH_UPDATE_INTERFACE.md only (no Control code imported but
the start/stop helpers of test/gazebo_path_update.py).
"""

import argparse
import glob
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray, String

from blueboat_interfaces.srv import UpdatePath

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "test"))
import gazebo_path_update as gz          # noqa: E402  (preflight / stop helpers)

SPEED = 0.4      # m/s
DT = 0.5         # s between samples


def straight(x0, y0, yaw, t0, length):
    n = int(round(length / (SPEED * DT)))
    t = [t0 + i * DT for i in range(n + 1)]
    s = [SPEED * (ti - t0) for ti in t]
    return (t, [x0 + d * math.cos(yaw) for d in s], [y0 + d * math.sin(yaw) for d in s],
            [math.atan2(math.sin(yaw), math.cos(yaw))] * len(t))


def left_arc(x0, y0, yaw, t0, radius, angle):
    cx, cy = x0 - radius * math.sin(yaw), y0 + radius * math.cos(yaw)
    n = int(round(radius * angle / (SPEED * DT)))
    t, x, y, h = [], [], [], []
    for i in range(n + 1):
        a = angle * i / n
        hh = yaw + a
        t.append(t0 + radius * a / SPEED)
        x.append(cx + radius * math.sin(hh))
        y.append(cy - radius * math.cos(hh))
        h.append(math.atan2(math.sin(hh), math.cos(hh)))
    return t, x, y, h


def concat(*parts):
    """Join path pieces that share their junction sample."""
    out = [list(v) for v in parts[0]]
    for p in parts[1:]:
        for k in range(4):
            out[k] += list(p[k])[1:]
    return out


def pose_at(path, tq):
    """The interface doc's evaluation rule: linear in t, yaw the short way."""
    t, x, y, yaw = path
    if tq <= t[0]:
        return x[0], y[0], yaw[0]
    if tq >= t[-1]:
        return x[-1], y[-1], yaw[-1]
    i = next(k for k in range(1, len(t)) if t[k] > tq)
    u = (tq - t[i - 1]) / (t[i] - t[i - 1])
    dyaw = math.atan2(math.sin(yaw[i] - yaw[i - 1]), math.cos(yaw[i] - yaw[i - 1]))
    return (x[i - 1] + u * (x[i] - x[i - 1]), y[i - 1] + u * (y[i] - y[i - 1]),
            yaw[i - 1] + u * dyaw)


def distance_to_polyline(px, py, path):
    _, x, y, _ = path
    best = math.inf
    for i in range(1, len(x)):
        ax, ay, bx, by = x[i - 1], y[i - 1], x[i], y[i]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        best = min(best, math.hypot(px - (ax + u * dx), py - (ay + u * dy)))
    return best


def to_path_msg(path):
    msg = Path()
    msg.header.frame_id = "world"
    for x, y, yaw in zip(path[1], path[2], path[3]):
        p = PoseStamped()
        p.header.frame_id = "world"
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.poses.append(p)
    return msg


def check_environment():
    """The nodes run on whatever `python3` is first on PATH (their shebang is
    `#!/usr/bin/env python3`): it must be the venv's, or master_control and
    simulation_interface die at import and the boat never moves."""
    probe = subprocess.run(["python3", "-c", "import casadi, acados_template"],
                           capture_output=True, text=True)
    if probe.returncode != 0:
        which = subprocess.run(["bash", "-c", "command -v python3"], capture_output=True,
                               text=True).stdout.strip()
        print(f"STOP: `python3` here is {which}, which cannot import casadi/acados_template, "
              "so master_control and simulation_interface would crash at launch and the "
              "boat would never move.\nActivate the venv in this terminal first:\n"
              "    source ~/ros2_ws/.venv/bin/activate\n"
              f"({probe.stderr.strip().splitlines()[-1] if probe.stderr.strip() else ''})")
        return False
    return True


class Client(Node):
    def __init__(self, log_path):
        super().__init__("quick_path_change_test")
        self.log_path = log_path
        self.last_check = 0.0
        self.followed = None        # what Control follows now (interface doc splice rule)
        self.refused = None
        self.pub_followed = self.create_publisher(Path, "/quick_path_change/followed_path", 1)
        self.pub_refused = self.create_publisher(Path, "/quick_path_change/refused_path", 1)
        self.cli = self.create_client(UpdatePath, "/blueboat/update_path")
        self.progress, self.boat, self.targets, self.events = None, None, [], []
        self.create_subscription(String, "/blueboat/path_progress",
                                 lambda m: setattr(self, "progress", json.loads(m.data)), 10)
        self.create_subscription(String, "/blueboat/path_update_event",
                                 lambda m: self.events.append(json.loads(m.data)),
                                 QoSProfile(depth=50))
        self.create_subscription(Odometry, "/blueboat/odom", self._odom, 10)
        self.create_subscription(Float32MultiArray, "/monitoring_data",
                                 lambda m: self.targets.append((m.data[4], m.data[5])), 10)

    def _odom(self, m):
        self.boat = (m.pose.pose.position.x, m.pose.pose.position.y)

    def housekeeping(self):
        """Once a second: fail fast if a launched node died, and republish
        the paths for RViz."""
        now = time.monotonic()
        if now - self.last_check < 1.0:
            return
        self.last_check = now
        with open(self.log_path, errors="replace") as handle:
            log = handle.read().splitlines()
        died = [line for line in log if "process has died" in line
                and any(n in line for n in ("master_control", "simulation_interface",
                                            "path_generation", "gazebo"))]
        if died:
            errors = [line for line in log if "Error" in line or "FATAL" in line][-6:]
            raise RuntimeError("a launched node crashed -- the boat cannot move:\n    "
                               + "\n    ".join(errors + died)
                               + f"\n  (full log: {self.log_path})")
        if self.followed:
            self.pub_followed.publish(to_path_msg(self.followed))
        if self.refused:
            self.pub_refused.publish(to_path_msg(self.refused))

    def spin_until(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.housekeeping()
            if cond():
                return
        raise TimeoutError(f"timed out waiting for {what}; last progress {self.progress}")

    def send(self, rid, path, change_point=float("nan")):
        req = UpdatePath.Request()
        req.id, req.change_point, req.frame = rid, float(change_point), "world"
        req.t, req.x, req.y, req.yaw = [list(map(float, v)) for v in path]
        fut = self.cli.call_async(req)
        self.spin_until(fut.done, 10.0, f"the answer to {rid}")
        r = fut.result()
        print(f"  {rid:<14} -> accepted={r.accepted}  reason={r.reason}\n"
              f"  {'':<14}    {r.message}", flush=True)
        return r


def scenario(node):
    results = []
    print("Waiting for Control (the MPC loads its solver; ~1 min if it must recompile) ...",
          flush=True)
    node.spin_until(lambda: node.cli.service_is_ready(), 180, "/blueboat/update_path")
    node.spin_until(lambda: node.progress and node.progress["state"] == "awaiting_path"
                    and node.progress["hold"], 180, "awaiting_path")
    hold = node.progress["hold"]
    print(f"Boat holding at ({hold['x']:.2f}, {hold['y']:.2f}), yaw {hold['yaw']:.2f} rad", flush=True)

    # PATH A -- first path: 30 m straight ahead, starting on the hold pose
    path_a = straight(hold["x"], hold["y"], hold["yaw"], 0.0, 30.0)
    results.append(("path A (first path) accepted", node.send("A-straight", path_a).accepted))
    node.followed = path_a

    t0 = time.monotonic()
    node.spin_until(lambda: time.monotonic() - t0 >= 10.0, 15, "10 s of following")
    print(f"After 10 s: tau={node.progress['tau']:.2f}, following "
          f"{node.progress['following_id']}, min_change_point="
          f"{node.progress['min_change_point']:.2f}", flush=True)

    # the change point: the earliest Control accepts now + 1 s of margin
    c = node.progress["min_change_point"] + 1.0
    xc, yc, yawc = pose_at(path_a, c)

    # PATH C -- INCOMPATIBLE: 1.5 m to the left of path A at c, turned 30 deg
    off = 1.5
    path_c = straight(xc - off * math.sin(yawc), yc + off * math.cos(yawc),
                      yawc + math.radians(30), c, 15.0)
    before = (node.progress["active_id"], node.progress["t_end"])
    rc = node.send("C-incompatible", path_c, change_point=c)
    node.refused = path_c
    rclpy.spin_once(node, timeout_sec=0.2)
    results.append(("path C refused", (not rc.accepted)
                    and rc.reason in ("discontinuous_position", "discontinuous_heading")))
    results.append(("refusal left path A in place",
                    (node.progress["active_id"], node.progress["t_end"]) == before))

    # PATH B -- COMPATIBLE: continues A at c, left 90 deg turn (r = 5 m), 10 m
    arc = left_arc(xc, yc, yawc, c, 5.0, math.pi / 2)
    tail = straight(arc[1][-1], arc[2][-1], arc[3][-1], arc[0][-1], 10.0)
    path_b = concat(arc, tail)
    rb = node.send("B-compatible", path_b, change_point=c)
    results.append(("path B accepted", rb.accepted))
    if rb.accepted:   # what Control now holds: A before c, then B
        keep = [i for i, t in enumerate(path_a[0]) if t < c]
        node.followed = [[path_a[k][i] for i in keep] + list(path_b[k]) for k in range(4)]

    node.spin_until(lambda: node.progress["following_id"] == "B-compatible", 60,
                    "path B to become active")
    k0 = len(node.targets)
    print(f"Path B active at tau={node.progress['tau']:.2f}; watching 30 s ...", flush=True)
    dists = []
    t1 = time.monotonic()
    while time.monotonic() - t1 < 30.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        node.housekeeping()
        if node.boat:
            dists.append(distance_to_polyline(node.boat[0], node.boat[1], path_b))
    steps = [math.hypot(b[0] - a[0], b[1] - a[1])
             for a, b in zip(node.targets[k0 - 40:], node.targets[k0 - 39:])]
    print(f"Boat distance to path B over 30 s: mean {sum(dists) / len(dists):.2f} m, "
          f"max {max(dists):.2f} m;  largest reference step around the junction "
          f"{max(steps):.3f} m", flush=True)
    results.append(("reference continuous at the junction (step < 0.1 m)", max(steps) < 0.1))
    results.append(("boat follows path B (within 1.5 m)", max(dists[100:]) < 1.5))
    activated = [e["id"] for e in node.events if e["event"] == "activated"]
    results.append(("activations A then B", activated == ["A-straight", "B-compatible"]))
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", default="MPC")
    ap.add_argument("--gui", action="store_true",
                    help="show Gazebo and RViz; keeps the simulation open until Enter")
    ap.add_argument("--data-dir", default=os.path.join(tempfile.gettempdir(),
                                                       "quick_path_change"))
    args = ap.parse_args()
    os.makedirs(args.data_dir, exist_ok=True)

    # The MCS empty-world simulation launch (launch_manager.to_cli) + path_source
    launch_args = ["robot_file:=thrusters_ur", "trajectory:=station_keeping",
                   f"controller_type:={args.controller}", "note:=quick_path_change",
                   "path_source:=replanner", f"data_dir:={args.data_dir}"]
    launch_args += ["rviz:=false"] if args.gui else ["gui:=false", "rviz:=false"]
    print("ros2 launch blueboat_control Sim_launch.py " + " ".join(launch_args))
    if not check_environment():
        return 2
    gz.preflight()
    log_path = os.path.join(args.data_dir, "launch.log")
    proc = subprocess.Popen(
        ["bash", "-c", "exec ros2 launch blueboat_control Sim_launch.py " + " ".join(launch_args)],
        stdout=open(log_path, "w"), stderr=subprocess.STDOUT, start_new_session=True)

    rviz = None
    if args.gui:   # Control's RViz view + this test's two paths
        rviz = subprocess.Popen(["rviz2", "-d", os.path.join(HERE, "quick_path_change.rviz")],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                start_new_session=True)

    rclpy.init()
    node = Client(log_path)
    try:
        results = scenario(node)
        if args.gui:
            print("\nChecks done -- the simulation keeps running. Press Enter to stop it.",
                  flush=True)
            end = time.monotonic() + 3600
            import select
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
                node.housekeeping()
                if select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    break
    except KeyboardInterrupt:
        results = [("scenario ran to the end (interrupted with Ctrl-C)", False)]
    except Exception as exc:                        # noqa: BLE001 - reported
        results = [(f"scenario ran to the end ({type(exc).__name__}: {exc})", False)]
        print(f"\n{exc}", flush=True)
    finally:
        # A second Ctrl-C here would orphan the simulation (its own session):
        # ignore it until everything is down.
        import signal
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print("Stopping the simulation and filing the run (takes ~10 s; Ctrl-C ignored) ...",
              flush=True)
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:                           # noqa: BLE001 - teardown
            pass
        if rviz is not None:
            rviz.terminate()
        gz.stop(proc)

    folders = sorted(glob.glob(os.path.join(args.data_dir, "data", "Robot_data", "*-poslog")),
                     key=os.path.getmtime)
    pathlog = glob.glob(os.path.join(folders[-1], "*-pathlog.csv")) if folders else []
    results.append(("run filed with its -pathlog.csv", len(pathlog) == 1))
    print()
    for name, ok in results:
        print(f"{'PASS' if ok else 'FAIL'}  {name}")
    print(f"\nRun folder: {folders[-1] if folders else '(none)'}\nLaunch log: {log_path}")
    return 0 if all(ok for _, ok in results) else 1


if __name__ == "__main__":
    sys.exit(main())
