#!/usr/bin/env python3
"""
Deterministic, tick-level harness for master_control.py -- no executor, no
Gazebo, no wall clock.

Two jobs:

  1. REGRESSION (`--compare HEAD`). Runs fixed-mission scenarios -- library
     shapes, manual target, arrow keys, run sessions -- for PID, LoS and MPC,
     once with the master_control.py of a git revision (its whole
     blueboat_control/src tree, extracted with `git archive`) and once with the
     working tree, and requires /thruster_input, /monitoring_data and tau to be
     BIT-IDENTICAL tick by tick. This is the proof that the live path update
     changes nothing when it is not used.

  2. LIVE PATH UPDATE, offline (`--replanner`). Drives a replanner-owned run of
     the working tree through the real service callback: hold, first path,
     updates (one mid-turn, back-to-back), a refused update, end of path,
     E-STOP, manual override, session stop / new run -- and checks each
     outcome; reports worst-case handling and per-tick cost.

How a run is made deterministic: the node's source is exec'd without its
module tail (rclpy.init / spin), `time` inside it is replaced by a simulated
clock advanced by control_dt per tick, /path_request is answered in-process by
path_generation.py of the SAME tree (one tick of latency, like the real
asynchronous request), odometry comes from a small 3-DOF plant integrating the
published /thruster_input, and every callback is called directly. Same inputs
-> same floats.

Needs a sourced ROS 2 + this workspace's install (blueboat_interfaces) and,
for MPC, the venv with acados:

    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    source ~/ros2_ws/.venv/bin/activate
    python3 test/mc_harness.py --compare HEAD
    python3 test/mc_harness.py --replanner
"""

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time as _real_time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))          # BlueBoat-Control/

CONTROLLERS = ("PID", "LoS", "MPC")


# ===========================================================================
#  The simulated world
# ===========================================================================
class FakeTime:
    """Stands in for the `time` module inside master_control."""

    def __init__(self, t0=1_000_000.0):
        self.now = float(t0)

    def time(self):
        return self.now

    def perf_counter(self):
        return _real_time.perf_counter()

    def sleep(self, _s):
        pass


class Plant:
    """3-DOF surge/sway/yaw boat, coefficients of the Gazebo plant
    (hydrodynamics.xacro, as fitted in master_control's simulation MPC model):
    deterministic, good enough for the controllers to close their loops."""

    M, IZ = 16.01, 5.64
    A_U, A_V, A_R = -5.50, -12.70, -0.12
    XU, XUU = 25.15, 33.8
    YV, YVV = 7.364, 54.269
    NR, NRR = 3.744, 40.0
    ARM = 0.295

    def __init__(self, x=0.0, y=0.0, psi=0.0):
        self.s = [x, y, psi, 0.0, 0.0, 0.0]          # x y psi u v r

    def step(self, thrust, dt, substeps=10):
        fr, fl = float(thrust[0]), float(thrust[1])
        X, N = fr + fl, self.ARM * (fr - fl)
        h = dt / substeps
        for _ in range(substeps):
            x, y, psi, u, v, r = self.s
            du = (X - (self.XU + self.XUU * abs(u)) * u) / (self.M - self.A_U)
            dv = (-(self.YV + self.YVV * abs(v)) * v) / (self.M - self.A_V)
            dr = (N - (self.NR + self.NRR * abs(r)) * r) / (self.IZ - self.A_R)
            u, v, r = u + h * du, v + h * dv, r + h * dr
            x += h * (u * math.cos(psi) - v * math.sin(psi))
            y += h * (u * math.sin(psi) + v * math.cos(psi))
            psi = math.atan2(math.sin(psi + h * r), math.cos(psi + h * r))
            self.s = [x, y, psi, u, v, r]

    def odometry(self):
        from nav_msgs.msg import Odometry
        x, y, psi, u, v, r = self.s
        msg = Odometry()
        msg.header.frame_id = 'world'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(psi / 2.0)
        msg.pose.pose.orientation.w = math.cos(psi / 2.0)
        msg.twist.twist.linear.x = u
        msg.twist.twist.linear.y = v
        msg.twist.twist.angular.z = r
        return msg


class Recorder:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    def get_subscription_count(self):
        return 1


class FakeFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class FakeClient:
    """/path_request answered in-process by the tree's own path_generation."""

    def __init__(self, server, response_type):
        self.server = server
        self.response_type = response_type
        self.calls = 0

    def wait_for_service(self, timeout_sec=None):
        return True

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls += 1
        return FakeFuture(self.server.generate_path(request, self.response_type()))


def _load(src_dir, filename, module_name):
    """Exec a node source without its module tail (rclpy.init / spin)."""
    path = os.path.join(src_dir, filename)
    with open(path) as handle:
        lines = handle.read().split("\n")
    cut = len(lines)
    for i in range(len(lines) - 1, -1, -1):
        if lines[i].startswith("rclpy.init()") or lines[i].startswith("def main("):
            cut = i
            break
    source = "\n".join(lines[:cut])
    namespace = {"__name__": module_name, "__file__": path}
    exec(compile(source, path, "exec"), namespace)
    return namespace


# ===========================================================================
#  One run in this process (called in a subprocess per scenario)
# ===========================================================================
def run_one(src_root, scenario, controller, real, out_path):
    src = os.path.join(src_root, "blueboat_control", "src")
    for sub in ("_custom_libraries", "PID", "MPC", ""):
        sys.path.insert(0, os.path.join(src, sub))
    import rclpy
    from std_msgs.msg import Bool, Float32MultiArray, String
    from blueboat_interfaces.srv import RequestPath

    sc = SCENARIOS[scenario]
    data_dir = tempfile.mkdtemp(prefix="mc_harness_")
    args = ["--ros-args", "--log-level", "error",
            "-p", f"controller_type:={controller}",
            "-p", f"simulation:={'false' if real else 'true'}",
            "-p", f"session_mode:={'true' if sc.get('session') else 'false'}",
            "-p", f"data_dir:={data_dir}",
            "-p", f"trajectory:={sc.get('trajectory', 'circle')}",
            "-p", "allow_duplicate_server:=true",
            "-p", "server_discovery_wait:=0.0"]
    if sc.get("path_source"):
        args += ["-p", f"path_source:={sc['path_source']}"]
    rclpy.init(args=args)

    pg_ns = _load(src, os.path.join("_custom_libraries", "path_generation.py"), "pg")
    server = pg_ns["PathGeneration"]()

    mc_ns = _load(src, "master_control.py", "mc")
    clock = FakeTime()
    mc_ns["time"] = clock

    class Harnessed(mc_ns["Controller"]):
        def create_client(self, srv_type, name, *a, **kw):
            if srv_type is RequestPath:
                return FakeClient(server, RequestPath.Response)
            return super().create_client(srv_type, name, *a, **kw)

    node = Harnessed()
    for attr in ("thruster_input_publisher", "data_publisher", "target_publisher",
                 "pose_arrow_publisher", "run_state_publisher", "path_progress_publisher",
                 "path_event_publisher"):
        if hasattr(node, attr):
            setattr(node, attr, Recorder())

    plant = Plant(*sc.get("start", (0.0, 0.0, 0.0)))
    dt = node.dt
    trace = {"thrust": [], "monitoring": [], "tau": [], "answers": [], "events": [],
             "tick_cost_ms": [], "progress": []}
    ctx = Ctx(node, plant, clock, trace, Bool, Float32MultiArray, String)

    n_ticks = sc["ticks"]
    on_tick = sc["on_tick_factory"]() if sc.get("on_tick_factory") else None
    if os.environ.get("MC_HARNESS_NO_GC"):
        # Timing runs: the harness's own trace lists grow by thousands of
        # objects; a generation-2 collection inside a timed tick is a harness
        # artefact, not a cost of the code under test.
        import gc
        gc.collect()
        gc.disable()
    for k in range(n_ticks):
        ctx.k = k
        for action in sc.get("actions", {}).get(k, []):
            action(ctx)
        if on_tick is not None:
            on_tick(ctx)
        if sc.get("arrow_stream") and sc["arrow_stream"](k):
            node.arrow_thrust_callback(Float32MultiArray(data=[6.0, 3.0]))
        node.odom_callback(plant.odometry())
        n_thr, n_mon = len(node.thruster_input_publisher.msgs), len(node.data_publisher.msgs)
        started = _real_time.perf_counter()
        node.timer_callback()
        trace["tick_cost_ms"].append((_real_time.perf_counter() - started) * 1e3)
        thr = node.thruster_input_publisher.msgs[n_thr:]
        mon = node.data_publisher.msgs[n_mon:]
        u = list(thr[-1].data) if thr else [0.0, 0.0]
        trace["thrust"].append([float(v) for v in u])
        trace["monitoring"].append([float(v) for v in mon[-1].data] if mon else None)
        trace["tau"].append(float(node.tau))
        if k % 2 == 0 and hasattr(node, "publish_path_progress"):
            node.publish_path_progress()
            trace["progress"].append(json.loads(node.path_progress_publisher.msgs[-1].data))
        plant.step(u, dt)
        clock.now += dt
    if hasattr(node, "path_event_publisher"):
        trace["events"] = [json.loads(m.data) for m in node.path_event_publisher.msgs]
    trace["final_state"] = plant.s
    trace["checks"] = [[n, bool(ok)] for n, ok in trace.get("checks", [])]
    trace["path_requests"] = getattr(node.client, "calls", None) if hasattr(node, "client") else None
    with open(out_path, "w") as handle:
        json.dump(trace, handle)
    node.destroy_node()
    server.destroy_node()
    rclpy.shutdown()


class Ctx:
    """What a scenario action can touch."""

    def __init__(self, node, plant, clock, trace, Bool, F32, String):
        self.node, self.plant, self.clock, self.trace = node, plant, clock, trace
        self.Bool, self.F32, self.String = Bool, F32, String
        self.k = 0
        self.seq = 0

    def ready(self, value=True):
        self.node.ready_callback(self.Bool(data=value))

    def manual(self, x, y):
        self.node.manual_target_callback(self.F32(data=[float(x), float(y)]))

    def arrows_release(self):
        self.node.arrow_thrust_callback(self.F32(data=[]))

    def run_command(self, **cmd):
        self.seq += 1
        cmd["seq"] = self.seq
        self.node.run_command_callback(self.String(data=json.dumps(cmd)))

    def update(self, rid, t, x, y, yaw, change_point=float("nan"), frame="world"):
        from blueboat_interfaces.srv import UpdatePath
        req = UpdatePath.Request()
        req.id, req.change_point, req.frame = rid, float(change_point), frame
        req.t, req.x, req.y, req.yaw = ([float(v) for v in t], [float(v) for v in x],
                                         [float(v) for v in y], [float(v) for v in yaw])
        resp = self.node.update_path_callback(req, UpdatePath.Response())
        self.trace["answers"].append({
            "k": self.k, "id": rid, "accepted": resp.accepted, "reason": resp.reason,
            "message": resp.message, "active_id": resp.active_id, "tau": resp.tau,
            "min_change_point": None if math.isnan(resp.min_change_point)
            else resp.min_change_point})
        return resp


# ===========================================================================
#  Scenarios
# ===========================================================================
def _at(**kw):
    """{tick: [actions]} from keyword-free (tick, fn) pairs."""
    return kw


def _ready(c):
    c.ready(True)


def _actions(*pairs):
    out = {}
    for tick, fn in pairs:
        out.setdefault(tick, []).append(fn)
    return out


SCENARIOS = {
    # --- fixed missions (regression) ---------------------------------------
    "circle": dict(trajectory="circle", ticks=1200, actions=_actions((0, _ready))),
    "kin_square": dict(trajectory="kin_square", ticks=1600, actions=_actions((0, _ready))),
    "fsin": dict(trajectory="fsin", ticks=1200, actions=_actions((0, _ready))),
    "manual": dict(trajectory="circle", ticks=1400, actions=_actions(
        (0, _ready), (300, lambda c: c.manual(6.0, 4.0)),
        (700, lambda c: c.manual(0.0, 0.0)),                    # resume sentinel
        (900, lambda c: c.manual(-3.0, 2.0)),
        (1100, lambda c: c.manual(0.0, 0.0)))),
    "arrow": dict(trajectory="kin_square", ticks=1200, actions=_actions(
        (0, _ready), (500, lambda c: c.arrows_release()),
        (800, lambda c: c.manual(3.0, 3.0))),
        # stream from 300 to 500 with a dead-man gap 400-430
        arrow_stream=lambda k: 300 <= k < 500 and not (400 <= k < 430)),
    "session": dict(session=True, trajectory="circle", ticks=1500, actions=_actions(
        (0, _ready),
        (40, lambda c: c.run_command(cmd="configure", controller_type=c.node.controller_type,
                                     use_pinger=False, note="h", mission="m")),
        (60, lambda c: c.run_command(cmd="start")),
        (300, lambda c: c.manual(4.0, -2.0)),
        (450, lambda c: c.manual(0.0, 0.0)),
        (700, lambda c: c.run_command(cmd="stop")),
        (800, lambda c: c.run_command(cmd="start")),
        (1000, lambda c: c.ready(False)),                       # E-STOP latched
        (1100, lambda c: c.ready(True)),                        # released
        (1300, lambda c: c.run_command(cmd="stop")))),
}

REGRESSION = ("circle", "kin_square", "fsin", "manual", "arrow", "session")

try:                                    # the live-path-update scenarios
    sys.path.insert(0, HERE)
    import mc_replanner_scenarios as _rs
    SCENARIOS.update(_rs.SCENARIOS)
except ImportError:                     # pragma: no cover
    pass


# ===========================================================================
#  Driver
# ===========================================================================
def _subprocess_run(src_root, scenario, controller, real, out):
    cmd = [sys.executable, os.path.abspath(__file__), "--one", "--src-root", src_root,
           "--scenario", scenario, "--controller", controller, "--out", out]
    if real:
        cmd.append("--real")
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"{scenario}/{controller} in {src_root} failed:\n"
                           f"{proc.stdout[-3000:]}\n{proc.stderr[-3000:]}")


def compare(rev, controllers, scenarios, real_too):
    tmp = tempfile.mkdtemp(prefix="mc_compare_")
    head_root = os.path.join(tmp, "head")
    os.makedirs(head_root)
    archive = subprocess.run(["git", "-C", REPO, "archive", rev, "blueboat_control/src"],
                             capture_output=True, check=True).stdout
    subprocess.run(["tar", "-x", "-C", head_root], input=archive, check=True)
    rows, failed = [], 0
    combos = [(s, c, False) for s in scenarios for c in controllers]
    if real_too:
        combos += [("kin_square", c, True) for c in controllers]
    for scenario, controller, real in combos:
        outs = []
        for label, root in (("head", head_root), ("work", REPO)):
            out = os.path.join(tmp, f"{label}-{scenario}-{controller}-{int(real)}.json")
            _subprocess_run(root, scenario, controller, real, out)
            with open(out) as handle:
                outs.append(json.load(handle))
        a, b = outs
        same_thr = a["thrust"] == b["thrust"]
        same_mon = a["monitoring"] == b["monitoring"]
        same_tau = a["tau"] == b["tau"]
        max_thr = max((abs(p - q) for ra, rb in zip(a["thrust"], b["thrust"])
                       for p, q in zip(ra, rb)), default=0.0)
        ok = same_thr and same_mon and same_tau and len(a["thrust"]) == len(b["thrust"])
        failed += 0 if ok else 1
        moving = sum(1 for r in a["thrust"] if r != [0.0, 0.0])
        rows.append((scenario + (" (real params)" if real else ""), controller,
                     len(a["thrust"]), moving, same_thr, same_mon, same_tau, max_thr,
                     max(b["tick_cost_ms"]) - max(a["tick_cost_ms"])))
        print(f"{rows[-1][0]:<26} {controller:<4} ticks={len(a['thrust']):5d} "
              f"non-zero-thrust={moving:5d}  thrust={'==' if same_thr else '!='}  "
              f"monitoring={'==' if same_mon else '!='}  tau={'==' if same_tau else '!='}  "
              f"max|dthrust|={max_thr:.3g}", flush=True)
    print(f"\n{len(rows) - failed}/{len(rows)} scenario runs bit-identical "
          f"({rev} vs working tree)")
    return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--compare", metavar="REV", help="regression: REV vs working tree")
    ap.add_argument("--replanner", action="store_true", help="offline path-update scenarios")
    ap.add_argument("--controllers", default=",".join(CONTROLLERS))
    ap.add_argument("--scenarios", default=",".join(REGRESSION))
    ap.add_argument("--no-real", action="store_true", help="skip the real-parameter runs")
    ap.add_argument("--one", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--src-root", default=REPO, help=argparse.SUPPRESS)
    ap.add_argument("--scenario", help=argparse.SUPPRESS)
    ap.add_argument("--controller", help=argparse.SUPPRESS)
    ap.add_argument("--real", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--out", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.one:
        run_one(args.src_root, args.scenario, args.controller, args.real, args.out)
        return 0
    controllers = [c for c in args.controllers.split(",") if c]
    if args.compare:
        return compare(args.compare, controllers, [s for s in args.scenarios.split(",") if s],
                       not args.no_real)
    if args.replanner:
        import mc_replanner_scenarios as rs
        return rs.main(controllers)
    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.path.insert(0, HERE)
    sys.exit(main())
