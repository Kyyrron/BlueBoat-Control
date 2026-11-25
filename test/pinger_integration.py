#!/usr/bin/env python3
"""
End-to-end check of the pinger logging chain on a real ROS graph (R7):

  fake MAVROS (odom, IMU, GPS at 10 Hz) + fake Water Linked (/uw_gps/fix at 4 Hz,
  /uw_gps_data at 2 Hz; a still pinger seen from a boat on a circle)
    -> pinger_estimator -> robot_interface (CSV pf_* / legacy columns, sidecar)
    -> SIGINT -> robot_interface's shutdown files the run (poslog_report);
  plus the live tuning file: created by the estimator, one value changed
  halfway, applied without restarting the estimate.

    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash   # (or an overlay)
    python3 test/pinger_integration.py [--mode log|follow] [--seconds 40]

Runs on a private ROS_DOMAIN_ID, signals only its own process groups, writes
under a temporary data root. Exit 0 when every check passes.
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
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "blueboat_control", "src", "_custom_libraries"))

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(("ok    " if cond else "FAIL  ") + name + ("" if cond else f"  {detail}"))


def feeder(seconds, tuning_file, heard, target=(12.0, 6.0)):
    """Fake MAVROS + Water Linked for `seconds`; halfway, one line of the live
    tuning file is uncommented and changed. `heard` collects (time, payload) of
    /blueboat/pinger/filter_config and /blueboat/pinger/estimate."""
    import rclpy
    from geometry_msgs.msg import Quaternion
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, NavSatFix
    from std_msgs.msg import Float32MultiArray, String

    rclpy.init()
    node = rclpy.create_node("pinger_integration_feeder")
    be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    odom_pub = node.create_publisher(Odometry, "/mavros/local_position/odom", be)
    imu_pub = node.create_publisher(Imu, "/mavros/imu/data", be)
    gps_pub = node.create_publisher(NavSatFix, "/mavros/global_position/global", be)
    fix_pub = node.create_publisher(String, "/uw_gps/fix", QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    pkt_pub = node.create_publisher(Float32MultiArray, "/uw_gps_data", 10)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic, key, qos in (("/blueboat/pinger/filter_config", "config", latched),
                            ("/blueboat/pinger/estimate", "estimate", 10)):
        node.create_subscription(String, topic, lambda m, k=key: heard[k].append(
            (time.time(), json.loads(m.data))), qos)
    t0 = time.time()
    edited = False
    seq = 0
    k = 0
    lat0, lon0 = 31.2639365, 130.2989003
    while time.time() - t0 < seconds:
        t = time.time() - t0
        w = 0.15
        x, y = 5.0 * math.cos(w * t), 5.0 * math.sin(w * t)
        yaw = w * t + math.pi / 2
        q = Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        o = Odometry()
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation = q
        o.twist.twist.linear.x = 5.0 * w
        o.twist.twist.angular.z = w
        odom_pub.publish(o)
        imu = Imu()
        imu.orientation = q
        imu.angular_velocity.z = w
        imu.linear_acceleration.z = 9.8
        imu_pub.publish(imu)
        g = NavSatFix()
        g.latitude = lat0 + math.degrees(y / 6378137.0)
        g.longitude = lon0 + math.degrees(x / (6378137.0 * math.cos(math.radians(lat0))))
        gps_pub.publish(g)
        if k % 2 == 0 and t > 1.0:          # 4 Hz Water Linked fix, measured 1.5 s ago
            tm = max(0.0, t - 1.5)
            xm, ym = 5.0 * math.cos(w * tm), 5.0 * math.sin(w * tm)
            yawm = w * tm + math.pi / 2
            dx, dy = target[0] - xm, target[1] - ym
            a = (math.cos(yawm) * dx + math.sin(yawm) * dy, -math.sin(yawm) * dx + math.cos(yawm) * dy, 0.4)
            seq += 1
            ep = {"t_req": t, "t_resp": t, "ok": True, "status": 200, "error": None,
                  "data": {"x": a[0], "y": a[1], "z": a[2], "std": 0.3, "position_valid": True}}
            fix_pub.publish(String(data=json.dumps({"schema": "blueboat_uwgps_fix/1", "seq": seq,
                                                    "cycle": "raw", "endpoints": {"raw": ep}})))
            if k % 4 == 0:
                pkt_pub.publish(Float32MultiArray(data=[2026.0, 11.0, 24.0, 0.0, 0.0, 0.0, 0.0,
                                                        a[0], a[1], a[2], 0.0, 0.0, 0.0, 0.0, 0.0, a[2],
                                                        a[0], a[1], a[2]]))
        if not edited and t > seconds / 2:
            text = open(tuning_file).read().replace("  # sigma_bearing_deg: 12.0 ", "  sigma_bearing_deg: 15.0 ")
            open(tuning_file, "w").write(text)
            heard["edit_t"] = time.time()
            edited = True
        k += 1
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


def start(cmd, env, log):
    return subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop(proc, wait=30.0):
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=wait)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=10)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--mode", choices=("log", "follow"), default="log")
    ap.add_argument("--seconds", type=float, default=40.0)
    args = ap.parse_args()
    root = tempfile.mkdtemp(prefix="pinger_integration_")
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("PINGER_TEST_DOMAIN", "91"),
               PYTHONDONTWRITEBYTECODE="1", ROS_HOME=os.path.join(root, "ros_home"))
    logs = open(os.path.join(root, "nodes.log"), "w")
    ri = start(["ros2", "run", "blueboat_control", "robot_interface.py", "--ros-args",
                "-p", "session_mode:=false", "-p", f"pinger_mode:={args.mode}",
                "-p", f"use_UWgps:={'true' if args.mode == 'follow' else 'false'}",
                "-p", f"data_dir:={root}", "-p", "note:=pinger_it", "-p", "use_sim_time:=false"], env, logs)
    tuning = os.path.join(root, "pinger_tuning.yaml")
    pe = start(["ros2", "run", "blueboat_control", "pinger_estimator.py", "--ros-args",
                "-p", f"pinger_filter_yaml:={tuning}"], env, logs)
    time.sleep(4.0)
    os.environ.update(ROS_DOMAIN_ID=env["ROS_DOMAIN_ID"])
    heard = {"config": [], "estimate": [], "edit_t": None}
    try:
        feeder(args.seconds, tuning, heard)
    finally:
        stop(pe)
        stop(ri, wait=60.0)
        logs.close()
    import robot_log_schema as rls
    import pinger_params as pp
    text = open(tuning).read()
    check("the tuning file was created with every parameter and its description",
          all(f" {n}: " in text for n in pp.load_schema()["parameters"]))   # one line was uncommented above
    after = [c for t, c in heard["config"] if heard["edit_t"] and t > heard["edit_t"]]
    check("a saved tuning-file edit is applied live (filter_config echo)",
          any(c.get("ok") and c["params"]["sigma_bearing_deg"] == 15.0 for c in after),
          [c.get("detail") for c in after])
    states = {e["status"] for t, e in heard["estimate"] if heard["edit_t"] and t > heard["edit_t"]}
    check("the estimate is kept across the live change (never re-initialising)",
          states and states <= {"tracking", "coasting"}, states)
    folders = [f for f in glob.glob(os.path.join(root, "data", "Robot_data", "*-poslog*")) if os.path.isdir(f)]
    check("the run was filed into its own folder", len(folders) == 1, folders)
    if not folders:
        print(open(os.path.join(root, "nodes.log")).read()[-3000:])
        return 1
    folder = folders[0]
    names = sorted(os.listdir(folder))
    stem = os.path.basename(folder)
    csv_path = os.path.join(folder, stem + ".csv")
    with open(csv_path, newline="") as fh:
        table = list(csv.reader(fh))
    _legend, hi = rls.split_header(table)
    header = table[hi]
    want = rls.columns_for_mode(args.mode)
    check(f"CSV layout is the {args.mode} layout ({len(want)} columns)", header == want, len(header))
    rows = [dict(zip(header, r)) for r in table[hi + 1:] if r]
    pf = [float(r["pf_x"]) for r in rows]
    finite = [v for v in pf if math.isfinite(v)]
    check("the filtered block is filled once the filter has initialised", len(finite) > len(rows) // 2,
          f"{len(finite)}/{len(rows)}")
    if finite:
        last = rows[-1]
        # robot_interface re-zeroes the pose to the first odom: the boat starts
        # at (5, 0) on its circle, so the pinger (12, 6) is (7, 6) in /blueboat/odom.
        err = math.hypot(float(last["pf_x"]) - 7.0, float(last["pf_y"]) - 6.0)
        check(f"filtered pinger at the true point at the end ({err:.2f} m)", err < 1.0, err)
        check("pf_status tracking at the end", float(last["pf_status"]) == 2.0, last["pf_status"])
    legacy = [(float(r["corrected_pinger_x"]), float(r["corrected_pinger_y"])) for r in rows]
    check("the legacy pinger columns are filled", any(abs(a) + abs(b) > 0 for a, b in legacy))
    if args.mode == "log":
        check("log layout keeps the path target columns", "target_x" in header)
    check("the report picture was rendered", any(n.endswith(".png") for n in names), names)
    sidecar = [n for n in names if n.endswith("-origin.yaml")]
    if sidecar:
        text = open(os.path.join(folder, sidecar[0])).read()
        check("the sidecar records the pinger mode and filter",
              f"pinger_mode: {args.mode}" in text and "pinger_filter_hash:" in text
              and "pinger_filter_params:" in text, text[-400:])
    print(f"{len(PASS)} passed, {len(FAIL)} failed  (run folder: {folder})")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
