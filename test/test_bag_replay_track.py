#!/usr/bin/env python3
"""
Tests of the bag replay's attitude and heave (2026-10-27):
_custom_libraries/replay_attitude.py (ROS-free) and bag_replay.Track.

    /usr/bin/python3 test/test_bag_replay_track.py            # ROS-free part
    source ~/ros2_ws/env.sh && python3 test/test_bag_replay_track.py
                                                              # + the recorded-bag part

The recorded-bag part runs on the HarbourMPC run bag (4 Hz attitude, four 25 Hz
bursts, roll std 6 deg) when it is on this machine and rclpy / rosbag2_py import;
it is skipped otherwise. BAG= overrides the run folder.
"""

import importlib.util
import math
import os
import subprocess
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "blueboat_control", "src")
sys.path.insert(0, os.path.join(SRC, "_custom_libraries"))
sys.path.insert(0, SRC)

import numpy as np                          # noqa: E402

import replay_attitude as ra                # noqa: E402

BAG = os.environ.get("BAG", os.path.expanduser(
    "~/ros2_ws/data/Robot_data/2026_10_27-16_10_14-HarbourMPC-poslog"))

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    if not cond:
        print(f"FAIL  {name}  {detail}")
        if "pytest" in sys.modules:
            raise AssertionError(f"{name}  {detail}")


def angle_deg(q0, q1):
    d = abs(float(np.dot(np.asarray(q0, float), np.asarray(q1, float))))
    return math.degrees(2.0 * math.acos(min(1.0, d)))


# ============================================================================
#  ROS-free
# ============================================================================

def test_rotations():
    rng = np.random.default_rng(1)
    r, p, y = rng.uniform(-0.6, 0.6, 50), rng.uniform(-0.6, 0.6, 50), rng.uniform(-3, 3, 50)
    r2, p2, y2 = ra.quat_to_rpy(ra.rpy_to_quat(r, p, y))
    check("rpy round trip", np.allclose([r, p, y], [r2, p2, y2], atol=1e-12))
    d = ra.euler_rates(0.0, 0.0, 0.1, 0.2, 0.3)
    check("euler rates at level attitude = body rates", np.allclose(d, (0.1, 0.2, 0.3)))
    # NED/FRD -> ENU/FLU: heading north (NED yaw 0) is ENU yaw pi/2, nose-up
    # pitch (NED +) is a negative FLU pitch, a right turn is a negative ENU yaw rate.
    out = ra.ned_frd_to_enu_flu(0.1, 0.2, 0.0, 0.01, 0.02, 0.03)
    check("ned->enu", np.allclose([float(v) for v in out],
                                  [0.1, -0.2, math.pi / 2, 0.01, -0.02, -0.03]))


def test_hermite_beats_slerp_on_a_synthetic_roll():
    """0.8 Hz +-15 deg roll and +-8 deg pitch sampled at 4 Hz with their rates."""
    f, ar, ap = 0.8, math.radians(15), math.radians(8)
    t = np.arange(0.0, 20.0, 0.25)
    roll, pitch, yaw = ar * np.sin(2 * np.pi * f * t), ap * np.cos(2 * np.pi * f * t), 0.3 + 0.05 * t
    droll = ar * 2 * np.pi * f * np.cos(2 * np.pi * f * t)
    dpitch = -ap * 2 * np.pi * f * np.sin(2 * np.pi * f * t)
    dyaw = np.full_like(t, 0.05)
    # Euler rates -> body rates (inverse ZYX kinematics).
    p = droll - np.sin(pitch) * dyaw
    q = np.cos(roll) * dpitch + np.sin(roll) * np.cos(pitch) * dyaw
    r = -np.sin(roll) * dpitch + np.cos(roll) * np.cos(pitch) * dyaw
    t_ns = (t * 1e9).astype(np.int64)
    quats = ra.rpy_to_quat(roll, pitch, yaw)
    trk = ra.AttitudeTrack(t_ns, quats, np.stack([p, q, r], 1))
    check("rates used as tangents", trk.tangents == "rates")
    check("rate_hz", abs(trk.rate_hz - 4.0) < 1e-6, trk.rate_hz)
    check("hits the samples", angle_deg(trk.at(int(t_ns[7])), quats[7]) < 1e-4)
    tt = np.arange(1.0, 19.0, 0.01)
    truth = ra.rpy_to_quat(ar * np.sin(2 * np.pi * f * tt), ap * np.cos(2 * np.pi * f * tt), 0.3 + 0.05 * tt)
    e_h, e_s = [], []
    for k, ti in enumerate(tt):
        tn = int(ti * 1e9)
        e_h.append(angle_deg(trk.at(tn), truth[k]))
        i = int(np.searchsorted(t_ns, tn))
        a = (tn - t_ns[i - 1]) / (t_ns[i] - t_ns[i - 1])
        q0, q1 = quats[i - 1], quats[i]
        e_s.append(angle_deg(_slerp(q0, q1, a), truth[k]))
    check("hermite << slerp (synthetic)", max(e_h) < 0.25 * max(e_s),
          f"hermite max {max(e_h):.3f} deg, slerp max {max(e_s):.3f} deg")
    check("hermite max < 0.5 deg", max(e_h) < 0.5, max(e_h))
    nod = ra.AttitudeTrack(t_ns, quats)
    check("finite-difference tangents without rates", nod.tangents == "differences")
    check("ends held", angle_deg(trk.at(-10), quats[0]) < 1e-4 and angle_deg(trk.at(10 ** 12), quats[-1]) < 1e-4)


def _slerp(q0, q1, a):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    th = math.acos(min(1.0, d))
    if th < 1e-9:
        return q0
    return (math.sin((1 - a) * th) * q0 + math.sin(a * th) * q1) / math.sin(th)


def test_yaw_wraps():
    t_ns = np.array([0, 250_000_000], np.int64)
    quats = ra.rpy_to_quat(np.zeros(2), np.zeros(2), np.array([math.pi - 0.05, -math.pi + 0.05]))
    trk = ra.AttitudeTrack(t_ns, quats, np.array([[0, 0, 0.4], [0, 0, 0.4]]))
    _, _, y = ra.quat_to_rpy(trk.at(125_000_000))
    check("yaw interpolates across +-pi", abs(abs(float(y[0])) - math.pi) < 0.01, float(y[0]))


def test_long_gap_is_linear():
    t_ns = np.array([0, 2_000_000_000], np.int64)
    quats = ra.rpy_to_quat(np.array([0.0, 0.2]), np.zeros(2), np.zeros(2))
    trk = ra.AttitudeTrack(t_ns, quats, np.array([[5.0, 0, 0], [-5.0, 0, 0]]))
    r, _, _ = ra.quat_to_rpy(trk.at(1_000_000_000))
    check("gap > MAX_HERMITE_GAP_S: linear, no overshoot", abs(float(r[0]) - 0.1) < 1e-9, float(r[0]))


def test_boot_clock_mapping():
    rng = np.random.default_rng(3)
    boot_ms = np.arange(0, 120_000, 240) + 5_000_000
    offset_ns = 1_793_000_000_000_000_000 - 5_000_000 * 1_000_000
    drift = 20e-6                                     # 20 ppm crystal difference
    delay = rng.exponential(15e6, len(boot_ms)) + 3e6  # >= 3 ms WiFi transport
    recv = (boot_ms * 1_000_000 * (1 + drift) + offset_ns + delay).astype(np.int64)
    true = (boot_ms * 1_000_000 * (1 + drift) + offset_ns + 3e6).astype(np.int64)
    got = ra.boot_to_bag_ns(recv, boot_ms)
    err_ms = np.abs(got - true) / 1e6
    check("boot->bag time within 2 ms", err_ms.max() < 2.0, err_ms.max())
    check("boot->bag beats receive time", np.std(got - true) < 0.2 * np.std(recv - true))
    # An autopilot reboot mid-bag: a second segment with its own offset.
    b2 = np.concatenate([boot_ms, np.arange(0, 60_000, 240) + 1000])
    r2 = np.concatenate([recv, recv[-1] + 2_000_000_000 + (np.arange(0, 60_000, 240) * 1_000_000)])
    got2 = ra.boot_to_bag_ns(r2, b2)
    check("reboot: monotonic across segments", np.all(np.diff(got2) > 0))


def test_heave_gate():
    t = np.arange(0.0, 60.0, 0.25)
    wave = 0.05 * np.sin(2 * np.pi * 0.7 * t) + 0.01 * t          # 5 cm heave on a drift
    z, st = ra.heave_gate(t, wave)
    check("plausible heave replayed", z is not None and st["std_m"] < 0.05, st)
    noise = np.cumsum(np.random.default_rng(0).normal(0, 0.4, len(t)))
    z, st = ra.heave_gate(t, noise)
    check("altitude noise pinned", z is None, st)
    check("parse_heave", (ra.parse_heave("auto"), ra.parse_heave(True), ra.parse_heave("False"))
          == ("auto", True, False))


# ============================================================================
#  The recorded bag (needs rclpy, rosbag2_py, ros_gz_interfaces)
# ============================================================================

def _load_bag_replay():
    try:
        import bag_replay                    # noqa: F401
    except ImportError as exc:
        return None, f"bag_replay not importable ({exc})"
    return sys.modules["bag_replay"], None


def _load_v11_bag_replay():
    """bag_replay.py as committed at HEAD (v1.1), as a module."""
    src = subprocess.run(["git", "-C", HERE, "show", "HEAD:blueboat_control/src/bag_replay.py"],
                         capture_output=True, text=True, check=True).stdout
    mod = types.ModuleType("bag_replay_v11")
    exec(compile(src, "bag_replay_v11", "exec"), mod.__dict__)
    return mod


def test_recorded_bag():
    br, why = _load_bag_replay()
    if br is None:
        print(f"skip  recorded bag: {why}")
        return
    if not os.path.isdir(BAG):
        print(f"skip  recorded bag: {BAG} absent")
        return
    bag_dir = br.resolve_bag(BAG)
    s, _ = br.read_track(bag_dir)
    check("bag has ATTITUDE in its raw mavlink", len(s["att_t"]) > 100, len(s["att_t"]))

    # 1. Decoded + converted ATTITUDE == what mavros published as imu/data.
    t_att, q_att, _ = br.mavlink_attitude(s)
    imu_t = np.asarray(s["imu_t"], np.int64)
    imu_q = np.asarray(s["imu_q"], float)
    recv = np.asarray(s["att_t"], np.int64)
    diffs = []
    for k in range(len(recv)):
        j = int(np.searchsorted(imu_t, recv[k]))
        cands = [i for i in (j - 1, j) if 0 <= i < len(imu_t)]
        i = min(cands, key=lambda i: abs(imu_t[i] - recv[k]))
        if abs(imu_t[i] - recv[k]) < 5_000_000:
            diffs.append(angle_deg(q_att[k], imu_q[i]))
    diffs = np.asarray(diffs)
    check("ATTITUDE -> ENU matches /mavros/imu/data (median < 0.1 deg)",
          len(diffs) > 100 and np.median(diffs) < 0.1, f"n={len(diffs)} median={np.median(diffs):.3f}")
    # Receive = FCU time + transport delay >= 0: the fastest deliveries sit on
    # the mapped time, the WiFi stalls (up to ~120 ms in this bag) above it.
    late_ms = (recv - t_att) / 1e6
    check("FCU time = lower envelope of the receive times",
          abs(np.percentile(late_ms, 5)) < 2.0 and 0.0 <= np.median(late_ms) < 10.0
          and late_ms.max() < 500.0,
          f"p5 {np.percentile(late_ms, 5):.1f} median {np.median(late_ms):.1f} max {late_ms.max():.1f} ms")

    # 2. Hermite vs slerp on the 25 Hz bursts, decimated to the 4 Hz the rest has.
    roll_t = t_att
    dense = np.nonzero(np.diff(roll_t) < 60_000_000)[0]
    runs, cur = [], [dense[0]]
    for i in dense[1:]:
        if i == cur[-1] + 1:
            cur.append(i)
        else:
            runs.append(cur)
            cur = [i]
    runs.append(cur)
    runs = [r for r in runs if len(r) >= 6]
    _, _, rates = br.mavlink_attitude(s)
    e_h, e_s = [], []
    for run in runs:
        idx = np.arange(run[0], run[-1] + 2)
        sel = [idx[0]]
        for j in idx[1:]:
            if roll_t[j] - roll_t[sel[-1]] >= 200_000_000:
                sel.append(j)
        sel = np.asarray(sel)
        trk = ra.AttitudeTrack(roll_t[sel], q_att[sel], rates[sel])
        for j in idx:
            if j in sel or roll_t[j] > roll_t[sel[-1]]:
                continue
            e_h.append(angle_deg(trk.at(int(roll_t[j])), q_att[j]))
            i = int(np.searchsorted(roll_t[sel], roll_t[j]))
            a = (roll_t[j] - roll_t[sel][i - 1]) / (roll_t[sel][i] - roll_t[sel][i - 1])
            e_s.append(angle_deg(_slerp(q_att[sel][i - 1], q_att[sel][i], a), q_att[j]))
    rms_h, rms_s = (float(np.sqrt(np.mean(np.square(e)))) for e in (e_h, e_s))
    print(f"      bag bursts: {len(e_h)} points, slerp RMS {rms_s:.2f} max {max(e_s):.2f} deg, "
          f"hermite RMS {rms_h:.2f} max {max(e_h):.2f} deg")
    check("hermite RMS < 0.7 deg on the bag", rms_h < 0.7, rms_h)
    check("hermite beats slerp on the bag", rms_h < 0.6 * rms_s, (rms_h, rms_s))

    # 3. The Track: mavlink source, heave pinned as noise, origin placement.
    trk = br.Track(s, heave="auto")
    check("Track attitude from mavlink", trk.attitude_source == "mavlink", trk.attitude_source)
    check("heave auto pins this bag", trk.heave_mode == "pinned_noisy", (trk.heave_mode, trk.heave_stats))
    x, y, z, q = trk.pose_at(trk.t0 + 40_000_000_000)
    check("pinned waterline", z == trk.waterline_z, z)
    check("origin placement: no offset", np.all(trk.world_offset == 0.0))
    rolls = [math.degrees(float(ra.quat_to_rpy(trk.pose_at(int(tn))[3])[0][0]))
             for tn in np.arange(trk.t0, trk.t[-1], 10_000_000)]
    check("replayed roll carries the recorded motion (std > 4 deg)", np.std(rolls) > 4.0, np.std(rolls))

    # 4. attitude_interp=slerp, heave=True: the v1.1 Track, bit for bit.
    try:
        v11 = _load_v11_bag_replay()
    except (subprocess.CalledProcessError, OSError) as exc:
        print(f"skip  v1.1 comparison: {exc}")
        return
    old = v11.Track(s, heave=True)
    new = br.Track(s, heave=True, attitude_source="imu" if len(s["imu_t"]) > len(s["odom_t"]) else "odom",
                   attitude_interp="slerp")
    same = all(old.pose_at(int(tn)) == new.pose_at(int(tn))
               for tn in np.arange(old.t0 - 10 ** 9, old.t[-1] + 10 ** 9, 37_000_000))
    check("slerp + heave=True == v1.1 bit for bit", same)


def main():
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print(f"{len(PASS)} passed, {len(FAIL)} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
