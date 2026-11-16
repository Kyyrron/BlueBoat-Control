#!/usr/bin/env python3
"""
Deterministic, tick-level harness for master_control.py -- no executor, no
Gazebo, no wall clock.

Two jobs:

  1. REGRESSION (`--compare HEAD`). Runs fixed-mission scenarios -- library
     shapes, manual target, arrow keys, run sessions, pinger following (PID and
     LoS) -- for PID, LoS and MPC, once with the master_control.py of a git
     revision (its whole blueboat_control/src tree, extracted with `git
     archive`) and once with the working tree, and requires /thruster_input,
     /monitoring_data, /controller_target and tau to be BIT-IDENTICAL tick by
     tick. The proof that a refactor (or the live path update, unused) changes
     nothing. `--scenarios replanner,replanner_session` compares those too.

  1b. CLOCK EQUIVALENCE (`--time-source ros`, 2026-11-07). Runs the same
     scenarios with the working tree twice -- `time_source` 'wall' (the
     default: `time.time()`) and 'ros' (the node clock, what every simulation
     launch passes so a run slowed below real time keeps one clock) -- both
     on one integer-nanosecond simulated clock, and requires the same four
     streams bit-identical: the parameter changes the clock's source, never
     what the node does with it.

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
    python3 test/mc_harness.py --time-source ros
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


class FakeTimeNs(FakeTime):
    """FakeTime on integer nanoseconds (the clock-equivalence runs): `time()`
    and the fake node clock below read the same integer, so 'wall' and 'ros'
    see the same floats."""

    def __init__(self, t0=1_000_000.0):
        self.ns = int(round(float(t0) * 1e9))

    @property
    def now(self):
        return self.ns * 1e-9

    @now.setter
    def now(self, value):
        self.ns = int(round(float(value) * 1e9))

    def time(self):
        return self.ns * 1e-9


class FakeNodeClock:
    """`node.get_clock()` for time_source 'ros': `now().nanoseconds`."""

    def __init__(self, ft):
        self._ft = ft

    def now(self):
        from rclpy.time import Time
        return Time(nanoseconds=self._ft.ns)


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


class IdentifiedPlant(Plant):
    """The identified BlueBoat plant (2026-11-13) as Gazebo integrates it with
    blueboat_description plant:=identified: hull of hydrodynamics_identified.xacro
    plus the actuator model simulation_interface applies. For offline checks of
    the field-equivalent configuration (`--plant identified`, or the two ends of
    the 95 % interval, `identified_slow` / `identified_fast`); never the
    `--compare` plant. docs/PARAMETER_REGISTER.md has every number's origin.

    Applied thrust per side: kappa * (T - kp * sqrt|T| * u).
      (m+A11) u' =  (m-A22) v r + X - Xuu u|u|
      (m+A22) v' = -(m-A11) u r - Yvv v|v|
       Izz    r' = (A22-A11) u v + arm (F_R - F_L) - Nr r - Nur |u| r
    (the sign of the added-mass coupling is the plugin's, gz-sim 8.15).
    """

    VARIANTS = {
        # identified values (surge: ident/bootstrap.json; yaw: ident/yaw_fit5.json; sway drag re-derived at
        # the CAD draft and Nr / Nur refitted with Izz held, 2026-11-15: 04b_control_r8/fit/yaw_fit5_r8_fixIzz.json)
        "identified":      dict(m=16.01, A11=1.60, A22=14.0, Izz=9.48, Xuu=4.23, Yvv=319.1,
                                Nr=6.35, Nur=5.12, kappa=0.493, kp=1.83),
        # The two ends below are session 04's (sway drag 108, Nr / Nur of its interval), not rebuilt on the
        # 2026-11-15 sway: register s8 (R4) states its numbers with them.
        # slow end of the 95 % interval: 23.7 N for 1 m/s instead of 20.2, the
        # largest surge and yaw inertia and yaw damping
        "identified_slow": dict(m=16.01, A11=10.87, A22=14.0, Izz=10.95, Xuu=4.96, Yvv=108.0,
                                Nr=7.10, Nur=15.1, kappa=0.493, kp=2.15),
        # fast end: 18.6 N for 1 m/s, the smallest inertias and yaw damping.
        # The smallest identified surge inertia (22.2 kg per commanded N) is
        # below the rigid mass at kappa 0.493, so this end carries kappa 0.793
        # and its force coefficients are scaled with it.
        "identified_fast": dict(m=16.01, A11=1.60, A22=14.0, Izz=13.04, Xuu=6.26, Yvv=108.0,
                                Nr=4.96, Nur=2.18, kappa=0.793, kp=1.68),
    }

    def __init__(self, x=0.0, y=0.0, psi=0.0, variant="identified"):
        super().__init__(x, y, psi)
        self.p = self.VARIANTS[variant]

    def step(self, thrust, dt, substeps=10):
        p = self.p
        m11, m22 = p["m"] + p["A11"], p["m"] + p["A22"]
        h = dt / substeps
        for _ in range(substeps):
            x, y, psi, u, v, r = self.s
            fr = p["kappa"] * (float(thrust[0]) - p["kp"] * math.sqrt(abs(float(thrust[0]))) * u)
            fl = p["kappa"] * (float(thrust[1]) - p["kp"] * math.sqrt(abs(float(thrust[1]))) * u)
            du = ((p["m"] - p["A22"]) * v * r + fr + fl - p["Xuu"] * u * abs(u)) / m11
            dv = (-(p["m"] - p["A11"]) * u * r - p["Yvv"] * v * abs(v)) / m22
            dr = ((p["A22"] - p["A11"]) * u * v + self.ARM * (fr - fl)
                  - p["Nr"] * r - p["Nur"] * abs(u) * r) / p["Izz"]
            u, v, r = u + h * du, v + h * dv, r + h * dr
            x += h * (u * math.cos(psi) - v * math.sin(psi))
            y += h * (u * math.sin(psi) + v * math.cos(psi))
            psi = math.atan2(math.sin(psi + h * r), math.cos(psi + h * r))
            self.s = [x, y, psi, u, v, r]


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
def run_one(src_root, scenario, controller, real, out_path, time_source=None):
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
    if sc.get("use_pinger"):
        args += ["-p", "use_pinger:=true"]
    if time_source == "ros":
        args += ["-p", "time_source:=ros"]
    # Extra parameter overrides (`--param k:=v`, or MC_HARNESS_PARAMS="k:=v;k:=v"):
    # the field-equivalent profile, per-run gains. A parameter the node does not
    # declare is ignored by ROS, so the same list can be given to an older tree.
    for override in [o for o in os.environ.get("MC_HARNESS_PARAMS", "").split(";") if o]:
        if real and override.startswith("gain_profile:="):
            continue        # the real-parameter runs ARE the real column; 'sim' is refused there
        args += ["-p", override]
    rclpy.init(args=args)

    pg_ns = _load(src, os.path.join("_custom_libraries", "path_generation.py"), "pg")
    server = pg_ns["PathGeneration"]()

    mc_ns = _load(src, "master_control.py", "mc")
    if os.environ.get("MC_HARNESS_NO_REBRANCH") and hasattr(mc_ns["ur_mpc"].MPCController, "rebranch_iterate"):
        # test_yaw_wrap.py: the solver as it was before the +-180 deg fix
        # "1" = both off; "rebranch" / "seed" = that one only
        which = os.environ["MC_HARNESS_NO_REBRANCH"]
        if which in ("1", "rebranch"):
            mc_ns["ur_mpc"].MPCController.rebranch_iterate = False
        if which in ("1", "seed"):
            mc_ns["ur_mpc"].MPCController.seed_cold_iterate = False
    clock = FakeTimeNs() if time_source else FakeTime()
    mc_ns["time"] = clock

    class Harnessed(mc_ns["Controller"]):
        def create_client(self, srv_type, name, *a, **kw):
            if srv_type is RequestPath:
                return FakeClient(server, RequestPath.Response)
            return super().create_client(srv_type, name, *a, **kw)

    node = Harnessed()
    if time_source == "ros":
        node.get_clock = lambda: FakeNodeClock(clock)
    for attr in ("thruster_input_publisher", "data_publisher", "target_publisher",
                 "pose_arrow_publisher", "run_state_publisher", "path_progress_publisher",
                 "path_event_publisher"):
        if hasattr(node, attr):
            setattr(node, attr, Recorder())

    variant = os.environ.get("MC_HARNESS_PLANT", "")
    if variant:
        plant = IdentifiedPlant(*sc.get("start", (0.0, 0.0, 0.0)), variant=variant)
    else:
        plant = Plant(*sc.get("start", (0.0, 0.0, 0.0)))
    dt = node.dt
    trace = {"thrust": [], "monitoring": [], "tau": [], "answers": [], "events": [],
             "tick_cost_ms": [], "progress": [], "controller_target": []}
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
        n_tgt = len(node.target_publisher.msgs)
        started = _real_time.perf_counter()
        node.timer_callback()
        trace["tick_cost_ms"].append((_real_time.perf_counter() - started) * 1e3)
        thr = node.thruster_input_publisher.msgs[n_thr:]
        mon = node.data_publisher.msgs[n_mon:]
        u = list(thr[-1].data) if thr else [0.0, 0.0]
        trace["thrust"].append([float(v) for v in u])
        trace["monitoring"].append([float(v) for v in mon[-1].data] if mon else None)
        trace["tau"].append(float(node.tau))
        trace["controller_target"].append(
            [[float(v) for v in m.data] for m in node.target_publisher.msgs[n_tgt:]])
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

def _pinger_feed_factory(goal=(8.0, 5.0)):
    """Feed /blueboat/pinger_coordinates every tick: the body-frame vector from
    the plant to a fixed world point, as robot_interface publishes it."""
    def feed(c):
        x, y, psi = c.plant.s[:3]
        dx, dy = goal[0] - x, goal[1] - y
        c.node.pinger_callback(c.F32(data=[math.cos(psi) * dx + math.sin(psi) * dy,
                                           -math.sin(psi) * dx + math.cos(psi) * dy, 0.0]))
    return feed


# Pinger following: PID and LoS only (MPC does not support it); covers the
# pinger branch, /controller_target and the LoS arrival latch.
SCENARIOS["pinger"] = dict(use_pinger=True, ticks=1200, controllers=("PID", "LoS"),
                           actions=_actions((0, _ready)),
                           on_tick_factory=_pinger_feed_factory)

REGRESSION = ("circle", "kin_square", "fsin", "manual", "arrow", "session", "pinger")

# One-off offline run of any route file: MC_HARNESS_ROUTE=<yaml>[:ticks[:x,y,psi]]
# registers the scenario "route" (used for the hull-interval mission durations).
if os.environ.get("MC_HARNESS_ROUTE"):
    _r = os.environ["MC_HARNESS_ROUTE"].split(":")
    SCENARIOS["route"] = dict(trajectory="from_yaml:" + _r[0],
                              ticks=int(_r[1]) if len(_r) > 1 else 6000,
                              start=tuple(float(v) for v in _r[2].split(",")) if len(_r) > 2 else (0.0, 0.0, 0.0),
                              actions=_actions((0, _ready)))

# Heading at +-180 deg (test_yaw_wrap.py; fixtures written by that test). Each
# west-going scenario has a twin rotated by pi that never comes near +-pi; the
# closed loop is rotation-equivariant, so the two thrust traces must agree.
_FIX = os.path.join(HERE, "fixtures")
for _name, _start in (("west_line", (0.0, 0.5, math.pi - 0.2)), ("east_line", (0.0, -0.5, -0.2)),
                      ("west_weave", (0.0, 0.0, math.pi)), ("east_weave", (0.0, 0.0, 0.0))):
    SCENARIOS["yaw_" + _name] = dict(
        trajectory="from_yaml:" + os.path.join(_FIX, "yaw_wrap_%s.yaml" % _name),
        ticks=1200, start=_start, actions=_actions((0, _ready)))

try:                                    # the live-path-update scenarios
    sys.path.insert(0, HERE)
    import mc_replanner_scenarios as _rs
    SCENARIOS.update(_rs.SCENARIOS)
except ImportError:                     # pragma: no cover
    pass


# ===========================================================================
#  Driver
# ===========================================================================
def _subprocess_run(src_root, scenario, controller, real, out, time_source=None):
    cmd = [sys.executable, os.path.abspath(__file__), "--one", "--src-root", src_root,
           "--scenario", scenario, "--controller", controller, "--out", out]
    if real:
        cmd.append("--real")
    if time_source:
        cmd += ["--one-time-source", time_source]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"{scenario}/{controller} in {src_root} failed:\n"
                           f"{proc.stdout[-3000:]}\n{proc.stderr[-3000:]}")


def compare(rev, controllers, scenarios, real_too, time_sources=None):
    """`rev` vs the working tree (time_sources None), or -- the clock
    equivalence -- the working tree under two time sources."""
    tmp = tempfile.mkdtemp(prefix="mc_compare_")
    if time_sources is None:
        head_root = os.path.join(tmp, "head")
        os.makedirs(head_root)
        archive = subprocess.run(["git", "-C", REPO, "archive", rev, "blueboat_control/src"],
                                 capture_output=True, check=True).stdout
        subprocess.run(["tar", "-x", "-C", head_root], input=archive, check=True)
        runs = (("head", head_root, None), ("work", REPO, None))
    else:
        runs = tuple((ts, REPO, ts) for ts in time_sources)
    rows, failed = [], 0
    combos = [(s, c, False) for s in scenarios for c in controllers
              if c in SCENARIOS[s].get("controllers", CONTROLLERS)]
    if real_too:
        combos += [("kin_square", c, True) for c in controllers]
    for scenario, controller, real in combos:
        outs = []
        for label, root, ts in runs:
            out = os.path.join(tmp, f"{label}-{scenario}-{controller}-{int(real)}.json")
            _subprocess_run(root, scenario, controller, real, out, ts)
            with open(out) as handle:
                outs.append(json.load(handle))
        a, b = outs
        same_thr = a["thrust"] == b["thrust"]
        same_mon = a["monitoring"] == b["monitoring"]
        same_tau = a["tau"] == b["tau"] and a["controller_target"] == b["controller_target"]
        max_thr = max((abs(p - q) for ra, rb in zip(a["thrust"], b["thrust"])
                       for p, q in zip(ra, rb)), default=0.0)
        ok = same_thr and same_mon and same_tau and len(a["thrust"]) == len(b["thrust"])
        failed += 0 if ok else 1
        moving = sum(1 for r in a["thrust"] if r != [0.0, 0.0])
        rows.append((scenario + (" (real params)" if real else ""), controller,
                     len(a["thrust"]), moving, same_thr, same_mon, same_tau, max_thr,
                     max(b["tick_cost_ms"]) - max(a["tick_cost_ms"])))
        note = ""
        if not ok:
            # Where a run stops being identical, and whether the measured yaw
            # had wrapped across +-180 deg by then (the 2026-11-13 MPC fix
            # changes a run from its first crossing, or from a run start away
            # from the zero state, and nowhere else).
            first = next((k for k in range(min(len(a["thrust"]), len(b["thrust"])))
                          if a["thrust"][k] != b["thrust"][k] or a["monitoring"][k] != b["monitoring"][k]), None)
            psi = [m[3] if m else None for m in a["monitoring"]]
            wrap = next((k for k in range(1, len(psi)) if psi[k] is not None and psi[k - 1] is not None
                         and abs(psi[k] - psi[k - 1]) > math.pi), None)
            note = f"  first-differing-tick={first}  first-yaw-wrap-tick={wrap}"
        print(f"{rows[-1][0]:<26} {controller:<4} ticks={len(a['thrust']):5d} "
              f"non-zero-thrust={moving:5d}  thrust={'==' if same_thr else '!='}  "
              f"monitoring={'==' if same_mon else '!='}  tau={'==' if same_tau else '!='}  "
              f"max|dthrust|={max_thr:.3g}{note}", flush=True)
    what = (f"{rev} vs working tree" if time_sources is None
            else " vs ".join(f"time_source {ts}" for ts in time_sources))
    print(f"\n{len(rows) - failed}/{len(rows)} scenario runs bit-identical ({what})")
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
    ap.add_argument("--time-source", choices=("ros",),
                    help="clock equivalence: the working tree with time_source wall vs ros")
    ap.add_argument("--one-time-source", help=argparse.SUPPRESS)
    ap.add_argument("--param", action="append", default=[], metavar="K:=V",
                    help="extra master_control parameter for every run (repeatable), "
                         "e.g. --param gain_profile:=sim")
    ap.add_argument("--plant", choices=tuple(IdentifiedPlant.VARIANTS), default=None,
                    help="offline plant for --one / --replanner runs (default: the legacy "
                         "BlueROV2 plant, the only one --compare uses)")
    args = ap.parse_args()
    if args.plant and not args.compare:
        os.environ["MC_HARNESS_PLANT"] = args.plant
    if args.param:
        os.environ["MC_HARNESS_PARAMS"] = ";".join(
            [o for o in os.environ.get("MC_HARNESS_PARAMS", "").split(";") if o] + args.param)
    if args.one:
        run_one(args.src_root, args.scenario, args.controller, args.real, args.out,
                args.one_time_source)
        return 0
    controllers = [c for c in args.controllers.split(",") if c]
    if args.compare:
        return compare(args.compare, controllers, [s for s in args.scenarios.split(",") if s],
                       not args.no_real)
    if args.time_source:
        return compare(None, controllers, [s for s in args.scenarios.split(",") if s],
                       not args.no_real, ("wall", args.time_source))
    if args.replanner:
        import mc_replanner_scenarios as rs
        return rs.main(controllers)
    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.path.insert(0, HERE)
    sys.exit(main())
