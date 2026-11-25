#!/usr/bin/env python3
"""
Gazebo test of how a path ENDS (R10, acceptance A5), per controller.

For each controller (PID, LoS, MPC) a headless Sim_launch flies a short straight
NON-LOOP path (12 m east at 0.4 m/s); after its end the boat must stay within
hold_radius + margin of the final point for at least --hold s and never head
back toward the start, with the target held at the final pose. One extra run
flies the same path with `loop: true` and checks the target wraps back to the
start.

Isolation (several sessions may share this machine): a private ROS_DOMAIN_ID,
GZ_PARTITION and ROS_HOME; each launch in its own process group, stopped with
one SIGINT; leftover processes are found by THIS run's GZ_PARTITION in their
environment and nothing else is ever signalled.

    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash   # or an overlay
    source ~/ros2_ws/.venv/bin/activate
    python3 test/gazebo_path_end.py --controllers PID,LoS,MPC --out-dir /tmp/gz_end
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
sys.path.insert(0, os.path.join(HERE, "..", "blueboat_control", "src", "_custom_libraries"))

import robot_log_schema as rls          # noqa: E402

SPEED, LENGTH = 0.4, 12.0
HOLD_RADIUS = 0.5                       # master_control's hold_radius default


def write_path(path, loop):
    n = int(LENGTH / SPEED / 0.25)
    with open(path, "w") as fh:
        fh.write("format: blueboat_trajectory/1\nname: path_end_test\nloop: %s\npoints:\n"
                 % ("true" if loop else "false"))
        for k in range(n + 1):
            t = 0.25 * k
            fh.write("- [%r, %r, 0.0, 0.0]\n" % (t, SPEED * t))
    return LENGTH / SPEED


def my_processes(partition):
    """PIDs whose environment carries this run's GZ_PARTITION (only ours)."""
    out = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open(f"/proc/{pid}/environ", "rb") as fh:
                env = fh.read().split(b"\0")
        except OSError:
            continue
        if f"GZ_PARTITION={partition}".encode() in env:
            out.append(int(pid))
    return out


def run_one(ctrl, yaml_path, duration_s, data_dir, env, log_path):
    cmd = ["ros2", "launch", "blueboat_control", "Sim_launch.py", "gui:=false", "rviz:=false",
           f"controller_type:={ctrl}", f"trajectory:=from_yaml:{yaml_path}", f"data_dir:={data_dir}",
           "note:=path_end"]
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(duration_s)
        os.kill(proc.pid, signal.SIGINT)          # one SIGINT to the launch (it files the run)
        try:
            proc.wait(timeout=90)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=20)
    for sig in (signal.SIGTERM, signal.SIGKILL):  # our own orphans only (gz sim leaves the group)
        for pid in my_processes(env["GZ_PARTITION"]):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(2.0)
    folders = [d for d in glob.glob(os.path.join(data_dir, "data", "Robot_data", "*")) if os.path.isdir(d)]
    if not folders:
        raise RuntimeError(f"{ctrl}: no run filed under {data_dir} (see {log_path})")
    csv_path = glob.glob(os.path.join(max(folders, key=os.path.getmtime), "*-poslog.csv"))[0]
    with open(csv_path, newline="") as fh:
        rows = list(csv.reader(fh))
    _, hi = rls.split_header(rows)
    header = rows[hi]
    data = [dict(zip(header, r)) for r in rows[hi + 1:] if r]
    t0 = None
    out = []
    for r in data:
        t = (float(r["Day"]) * 86400 + float(r["Hour"]) * 3600 + float(r["Minute"]) * 60
             + float(r["Second"]) + float(r["MicroSecond"]) * 1e-6)
        t0 = t if t0 is None else t0
        out.append((t - t0, float(r["relative_x"]), float(r["relative_y"]),
                    float(r["target_x"]), float(r["target_y"])))
    return out


def evaluate_end(ctrl, track, path_s, hold_s, margin):
    """Checks after the end of a non-loop path."""
    end = (LENGTH, 0.0)
    # the target reaches the end; the boat lags the governor -- wait for it to settle
    t_target_end = next((t for t, _x, _y, tx, ty in track if math.hypot(tx - end[0], ty - end[1]) < 1e-3), None)
    res = {"controller": ctrl, "target_reached_end_at_s": t_target_end}
    if t_target_end is None:
        res["ok"] = False
        res["why"] = "the target never reached the final point"
        return res
    settle = t_target_end + 15.0
    after = [s for s in track if s[0] >= settle]
    span = after[-1][0] - after[0][0] if after else 0.0
    dist = [math.hypot(x - end[0], y - end[1]) for _t, x, y, _tx, _ty in after]
    target_held = all(math.hypot(tx - end[0], ty - end[1]) < 1e-3 for _t, _x, _y, tx, ty in after)
    # "never heads back": along-path progress never drops below the end minus the hold band
    back = min((x for _t, x, _y, _tx, _ty in after), default=float("nan"))
    res.update(hold_span_s=span, max_dist_m=max(dist) if dist else None,
               median_dist_m=sorted(dist)[len(dist) // 2] if dist else None,
               min_progress_x_m=back, target_held=target_held)
    res["ok"] = (span >= hold_s and dist and max(dist) <= HOLD_RADIUS + margin and target_held
                 and back >= LENGTH - (HOLD_RADIUS + margin))
    return res


def evaluate_loop(track, path_s):
    """With loop: true the target wraps: back near the start after one duration."""
    later = [(t, tx, ty) for t, _x, _y, tx, ty in track if t > path_s + 5.0]
    near_start = [t for t, tx, ty in later if math.hypot(tx, ty) < 2.0]
    return {"target_back_near_start": bool(near_start), "first_at_s": near_start[0] if near_start else None,
            "ok": bool(near_start)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--controllers", default="PID,LoS,MPC")
    ap.add_argument("--out-dir", default="/tmp/gazebo_path_end")
    ap.add_argument("--hold", type=float, default=60.0, help="seconds the end must be held")
    ap.add_argument("--margin", type=float, default=1.0, help="m beyond hold_radius allowed")
    ap.add_argument("--no-loop", action="store_true", help="skip the loop-path run")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("PATH_END_DOMAIN", "94"),
               GZ_PARTITION=f"path_end_{os.getpid()}", ROS_HOME=os.path.join(args.out_dir, "ros_home"),
               PYTHONDONTWRITEBYTECODE="1")
    results = []
    path_file = os.path.join(args.out_dir, "path_end.yaml")
    path_s = write_path(path_file, loop=False)
    for ctrl in [c for c in args.controllers.split(",") if c]:
        data_dir = os.path.join(args.out_dir, f"end-{ctrl}")
        os.makedirs(data_dir, exist_ok=True)
        track = run_one(ctrl, path_file, path_s + 15.0 + args.hold + 45.0, data_dir, env,
                        os.path.join(data_dir, "launch.log"))
        res = evaluate_end(ctrl, track, path_s, args.hold, args.margin)
        results.append(res)
        print(json.dumps(res), flush=True)
    if not args.no_loop:
        loop_file = os.path.join(args.out_dir, "path_loop.yaml")
        write_path(loop_file, loop=True)
        data_dir = os.path.join(args.out_dir, "loop-PID")
        os.makedirs(data_dir, exist_ok=True)
        track = run_one("PID", loop_file, path_s + 40.0, data_dir, env, os.path.join(data_dir, "launch.log"))
        res = evaluate_loop(track, path_s)
        res["controller"] = "PID (loop: true)"
        results.append(res)
        print(json.dumps(res), flush=True)
    with open(os.path.join(args.out_dir, "path_end_summary.json"), "w") as fh:
        json.dump(results, fh, indent=1)
    ok = all(r["ok"] for r in results)
    print("PATH END: " + ("ALL PASS" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
