#!/usr/bin/env python3
"""
Gazebo tests of the live path update, and a Gazebo before/after regression of a
fixed mission. Runs headless (gui:=false); one Sim_launch per run, in its own
process group, shut down with SIGINT so the run is filed like any other.

  --replanner   per controller: Sim_launch path_source:=replanner + the client
                test/path_update_demo.py (first path from the hold, 5 clean
                updates incl. one mid-turn and two back-to-back, one refused,
                end of path), then checks on the FILED artifacts: the run folder
                holds the -pathlog.csv with the expected rows, and the poslog's
                target trail never jumps at a junction.
  --regression  per controller: kin_square for --duration s with the overlay
                given by --head-setup (a workspace built from HEAD) and twice
                with the current install, comparing the logged tracks: the
                HEAD-vs-new difference against the new-vs-new noise floor
                (Gazebo is not bit-deterministic; the bit-exact proof is
                test/mc_harness.py --compare).

    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    source ~/ros2_ws/.venv/bin/activate
    python3 test/gazebo_path_update.py --replanner --out-dir /tmp/gz
"""

import argparse
import csv
import glob
import json
import math
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(REPO, "blueboat_control", "src", "_custom_libraries"))

import path_update_log as pl            # noqa: E402
import robot_log_schema as rls          # noqa: E402


LEFTOVER_EXECUTABLES = {"master_control.py", "simulation_interface.py", "path_generation.py",
                        "path_publisher.py", "parameter_bridge", "pose_to_tf", "rviz2",
                        "robot_state_publisher"}


def _is_leftover(argv):
    """Matched on the EXECUTABLE (or the script a python/ruby interpreter
    runs), never on free text anywhere in a command line: a shell, an editor
    or a grep that merely mentions 'rviz2' is not a running RViz."""
    names = [os.path.basename(a) for a in argv[:2]]
    if names[0] in LEFTOVER_EXECUTABLES:
        return True
    if names[0].startswith("python") and names[1:2] and names[1] in LEFTOVER_EXECUTABLES:
        return True
    if names[0] == "gz" and argv[1:2] == ["sim"]:                      # gz sim
        return True
    if names[0].startswith("ruby") and names[1:2] == ["gz"] and argv[2:3] == ["sim"]:
        return True
    return (len(names) > 1 and names[1] == "ros2" and argv[2:3] == ["launch"]
            and "Sim_launch.py" in argv)                                # ros2 launch


def leftovers():
    out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True).stdout
    found = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and int(parts[0]) != os.getpid() and _is_leftover(parts[1:]):
            found.append(line.strip())
    return found


def preflight():
    """Two graphs at once would put two master_control on one service and
    two producers on /thruster_input: refuse rather than measure that."""
    left = leftovers()
    if left:
        raise RuntimeError("simulation/control processes are already running -- stop "
                           "them first:\n  " + "\n  ".join(left))


def launch(args, log_path, env_prefix=""):
    preflight()
    cmd = ("ros2 launch blueboat_control Sim_launch.py gui:=false rviz:=false "
           + " ".join(args))
    # exec: the launch must BE the process we signal (stop()), not a child of bash
    cmd = f"{env_prefix} && exec {cmd}" if env_prefix else f"exec {cmd}"
    log = open(log_path, "w")
    return subprocess.Popen(["bash", "-c", cmd],
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop(proc, grace=90.0):
    """SIGINT the LAUNCH process only -- an operator's Ctrl-C. ros2 launch
    forwards it to every node once; signalling the whole group as well would
    deliver a second SIGINT in the middle of simulation_interface's shutdown,
    killing it before it files the run (measured: exit -2, nothing filed).
    gz sim's server leaves the group, so orphans are cleaned up afterwards."""
    try:
        os.kill(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        os.kill(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=10)
    deadline = time.monotonic() + 10.0
    while leftovers() and time.monotonic() < deadline:
        time.sleep(1.0)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for line in leftovers():
            try:
                os.kill(int(line.split()[0]), sig)
            except (ProcessLookupError, ValueError):
                pass
        time.sleep(3.0)
    left = leftovers()
    if left:
        raise RuntimeError("processes survived the shutdown:\n  " + "\n  ".join(left))


def read_poslog(path):
    with open(path) as handle:
        rows = list(csv.reader(handle))
    _, start = rls.split_header(rows)
    header = rows[start]
    return [dict(zip(header, r)) for r in rows[start + 1:] if r]


def run_folder(data_dir):
    folders = [d for d in glob.glob(os.path.join(data_dir, "data", "Robot_data", "*"))
               if os.path.isdir(d)]
    return max(folders, key=os.path.getmtime) if folders else None


# ---------------------------------------------------------------------------
def replanner(controllers, out_dir, timeout):
    failures = 0
    summary = []
    for ctrl in controllers:
        data_dir = os.path.join(out_dir, f"replanner-{ctrl}")
        os.makedirs(data_dir, exist_ok=True)
        proc = launch([f"controller_type:={ctrl}", "path_source:=replanner",
                       f"data_dir:={data_dir}"], os.path.join(data_dir, "launch.log"))
        demo_out = os.path.join(data_dir, "demo.json")
        try:
            rc = subprocess.run([sys.executable, os.path.join(HERE, "path_update_demo.py"),
                                 "--out", demo_out, "--timeout", str(timeout)],
                                timeout=timeout + 120).returncode
        finally:
            stop(proc)
        with open(demo_out) as handle:
            demo = json.load(handle)
        checks = [(n, ok) for n, ok in demo["checks"]]
        folder = run_folder(data_dir)
        pathlogs = glob.glob(os.path.join(folder, "*-pathlog.csv")) if folder else []
        checks.append(("run filed with its -pathlog.csv", len(pathlogs) == 1))
        if pathlogs:
            _, _, rows = pl.read_pathlog(pathlogs[0])
            kinds = [r["event"] for r in rows]
            checks.append(("pathlog: 6 accepted, 1 refused, 5 activated, 1 end_reached",
                           (kinds.count("request_accepted"), kinds.count("request_refused"),
                            kinds.count("activated"), kinds.count("end_reached"))
                           == (6, 1, 5, 1)))
            handling = [float(r["handling_ms"]) for r in rows if r["handling_ms"]]
        else:
            handling = []
        poslogs = glob.glob(os.path.join(folder, "*-poslog.csv")) if folder else []
        steps, track = [], {}
        if poslogs:
            rows = read_poslog(poslogs[0])
            pts = [(float(r["relative_x"]), float(r["relative_y"]),
                    float(r["target_x"]), float(r["target_y"])) for r in rows]
            # the log timer is 0.33 s: a target moving at 0.4 m/s steps 0.13 m;
            # anything beyond 0.4 m between rows is a jump
            for a, b in zip(pts, pts[1:]):
                if (a[2], a[3]) != (0.0, 0.0) and (b[2], b[3]) != (0.0, 0.0):
                    steps.append(math.hypot(b[2] - a[2], b[3] - a[3]))
            gaps = [math.hypot(p[0] - p[2], p[1] - p[3]) for p in pts
                    if (p[2], p[3]) != (0.0, 0.0)]
            track = {"rows": len(pts), "boat_target_median_m": sorted(gaps)[len(gaps) // 2],
                     "boat_target_max_m": max(gaps)}
            checks.append(("poslog target trail never jumps (> 0.4 m per log row)",
                           max(steps) <= 0.4))
        failed = [n for n, ok in checks if not ok]
        failures += len(failed)
        rt = [a["round_trip_ms"] for a in demo.get("answers", [])]
        line = {"controller": ctrl, "checks": f"{len(checks) - len(failed)}/{len(checks)}",
                "failed": failed, "error": demo.get("error"), "demo_exit": rc,
                "handling_ms_max": max(handling) if handling else None,
                "service_round_trip_ms_max": max(rt) if rt else None,
                "max_target_step_per_log_row_m": max(steps) if steps else None,
                **track, "folder": folder}
        summary.append(line)
        print(json.dumps(line), flush=True)
    with open(os.path.join(out_dir, "replanner-summary.json"), "w") as handle:
        json.dump(summary, handle, indent=1)
    print("ALL PASSED" if not failures else f"{failures} FAILED")
    return 1 if failures else 0


# ---------------------------------------------------------------------------
def regression(controllers, out_dir, duration, head_setup):
    """kin_square, fixed mission: HEAD build vs current build vs current again."""
    results = []
    for ctrl in controllers:
        tracks = {}
        for label, prefix in (("head", f"source {head_setup}"), ("new", ""), ("new2", "")):
            data_dir = os.path.join(out_dir, f"regr-{ctrl}-{label}")
            os.makedirs(data_dir, exist_ok=True)
            env = prefix
            proc = launch([f"controller_type:={ctrl}", "trajectory:=kin_square",
                           f"data_dir:={data_dir}"], os.path.join(data_dir, "launch.log"),
                          env_prefix=env)
            time.sleep(duration)
            stop(proc)
            folder = run_folder(data_dir)
            rows = read_poslog(glob.glob(os.path.join(folder, "*-poslog.csv"))[0])
            tracks[label] = rows
        results.append(_compare_tracks(ctrl, tracks))
        print(json.dumps(results[-1]), flush=True)
    with open(os.path.join(out_dir, "regression-summary.json"), "w") as handle:
        json.dump(results, handle, indent=1)
    return 0


def _series(rows):
    """(t, x, y, tx, ty, thr_r, thr_l) from the first row whose target is set,
    t measured from there (simulated time, from the date columns)."""
    out = []
    t0 = None
    for r in rows:
        if (float(r["target_x"]), float(r["target_y"])) == (0.0, 0.0):
            continue
        t = (float(r["Hour"]) * 3600 + float(r["Minute"]) * 60 + float(r["Second"])
             + float(r["MicroSecond"]) * 1e-6)
        t0 = t if t0 is None else t0
        out.append((t - t0, float(r["relative_x"]), float(r["relative_y"]),
                    float(r["target_x"]), float(r["target_y"]),
                    float(r["right_thr_in"]), float(r["left_thr_in"])))
    return out


def _interp(series, t, k):
    for a, b in zip(series, series[1:]):
        if a[0] <= t <= b[0]:
            u = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
            return a[k] + u * (b[k] - a[k])
    return None


def _diff(sa, sb, horizon):
    """RMS / max of the position difference of two runs, on a common time grid."""
    grid = [i * 0.5 for i in range(int(horizon / 0.5))]
    d = []
    for t in grid:
        xa, ya, xb, yb = (_interp(sa, t, 1), _interp(sa, t, 2), _interp(sb, t, 1),
                          _interp(sb, t, 2))
        if None not in (xa, ya, xb, yb):
            d.append(math.hypot(xa - xb, ya - yb))
    return (math.sqrt(sum(v * v for v in d) / len(d)) if d else None, max(d) if d else None,
            len(d))


def _compare_tracks(ctrl, tracks):
    s = {k: _series(v) for k, v in tracks.items()}
    horizon = min(v[-1][0] for v in s.values())
    head_new = _diff(s["head"], s["new"], horizon)
    new_new = _diff(s["new"], s["new2"], horizon)

    def gap(series):
        g = [math.hypot(p[1] - p[3], p[2] - p[4]) for p in series]
        return sum(g) / len(g)
    return {"controller": ctrl, "compared_s": round(horizon, 1),
            "head_vs_new_rms_m": head_new[0], "head_vs_new_max_m": head_new[1],
            "new_vs_new_rms_m (noise floor)": new_new[0], "new_vs_new_max_m": new_new[1],
            "mean_boat_target_m": {k: gap(v) for k, v in s.items()}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--replanner", action="store_true")
    ap.add_argument("--regression", action="store_true")
    ap.add_argument("--controllers", default="PID,LoS,MPC")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--timeout", type=float, default=900.0)
    ap.add_argument("--duration", type=float, default=150.0)
    ap.add_argument("--head-setup", help="setup.bash of a workspace built from HEAD")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    controllers = [c for c in args.controllers.split(",") if c]
    rc = 0
    if args.replanner:
        rc |= replanner(controllers, args.out_dir, args.timeout)
    if args.regression:
        if not args.head_setup:
            ap.error("--regression needs --head-setup")
        rc |= regression(controllers, args.out_dir, args.duration, args.head_setup)
    return rc


if __name__ == "__main__":
    sys.exit(main())
