#!/usr/bin/env python3

"""
Attitude and heave maths of the bag replay (bag_replay.py), ROS-free.

WHY. A run bag records the autopilot's attitude at its MAVLink stream rate --
4 Hz with ArduPilot's defaults, about 13 Hz when the sonar driver requests
SonarView's rates -- while the boat rolls and pitches ±10-20 deg at 0.6-2 Hz in
harbour chop (run 2026_10_27-16_10_14-HarbourMPC). The replay moves the
simulated boat, and a simulated sonar renders every ping from that pose, so
the pose between two recorded samples matters:

  * the samples' TIMES: mavros stamps are 0 on a real launch (use_sim_time
    without /clock) and receive times jitter ~11 ms over WiFi, but the bag's
    raw /uas1/mavlink_source holds every ATTITUDE with the autopilot's own
    time_boot_ms -- mapped here onto bag time (`boot_to_bag_ns`);
  * the INTERPOLATION: a slerp between 4 Hz samples cuts every roll peak
    (1.19 deg RMS / 4.4 deg max error, measured on that bag against its 25 Hz
    bursts); a cubic Hermite on the Euler angles whose tangents are the
    RECORDED body rates halves it (0.53 / 1.8 deg) -- `AttitudeTrack`;
  * the HEAVE: the EKF's z is baro/GPS altitude that drifts metres; its
    detrended residue is replayed only when it looks like motion, not noise
    (`heave_gate`).

Conventions: quaternions (x, y, z, w); world ENU, body FLU (as mavros
publishes); Euler angles ZYX (yaw, pitch, roll), radians; times ns (int).

    >>> import replay_attitude as ra
    >>> trk = ra.AttitudeTrack(t_ns, quats, body_rates)
    >>> trk.at(t)          # (qx, qy, qz, qw)
"""

import struct

import numpy as np

ATTITUDE_MSGID = 30

# Hermite only across gaps shorter than this: over a longer gap the recorded
# rates no longer describe the motion in between, and the cubic can overshoot.
MAX_HERMITE_GAP_S = 0.6

HEAVE_WINDOW_S = 10.0
HEAVE_CLIP_M = 0.5
HEAVE_MAX_STD_M = 0.10        # a BlueBoat in chop heaves centimetres, not decimetres
HEAVE_MAX_CLIP_FRACTION = 0.05


# ============================================================================
#  MAVLink ATTITUDE
# ============================================================================

def mavlink_payload(payload64, length):
    """The message payload bytes of a mavros_msgs/Mavlink, zero-extended.

    MAVLink 2 truncates trailing zero bytes; the receiver pads them back."""
    raw = b''.join(struct.pack('<Q', int(w) & 0xFFFFFFFFFFFFFFFF) for w in payload64)
    return raw[:int(length)] + b'\0' * 64


def decode_attitude(payload):
    """ATTITUDE (#30): (time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed,
    yawspeed), autopilot frame (NED world, FRD body), rad and rad/s."""
    return struct.unpack('<I6f', payload[:28])


def ned_frd_to_enu_flu(roll, pitch, yaw, p, q, r):
    """Euler angles and body rates, NED/FRD -> ENU/FLU (what mavros publishes):
    roll kept, pitch and the two body rates about y and z negated, yaw measured
    from East counter-clockwise."""
    roll, pitch, yaw = (np.asarray(v, float) for v in (roll, pitch, yaw))
    return (roll, -pitch, np.pi / 2.0 - yaw,
            np.asarray(p, float), -np.asarray(q, float), -np.asarray(r, float))


def boot_to_bag_ns(recv_ns, boot_ms, window_s=10.0, lower_pct=5.0):
    """Bag-time stamps (ns) of autopilot samples, from their time_boot_ms.

    The receive time is boot time + a constant offset + a positive transport
    delay; the delay's lower envelope (the fastest deliveries) carries the
    offset. Per window_s window its lower_pct percentile is taken, a line is
    fitted through them (the two crystals drift a few ppm), and a reboot of
    the autopilot (time_boot_ms going backwards) starts a new segment."""
    recv = np.asarray(recv_ns, np.int64)
    boot = np.asarray(boot_ms, np.int64) * 1_000_000
    out = np.empty(len(recv), np.int64)
    starts = [0] + [i + 1 for i in np.nonzero(np.diff(boot) < 0)[0]] + [len(recv)]
    for a, b in zip(starts[:-1], starts[1:]):
        bt, off = boot[a:b], (recv[a:b] - boot[a:b]).astype(float)
        edges = np.arange(bt[0], bt[-1] + 1, int(window_s * 1e9))
        cx, cy = [], []
        for lo in edges:
            sel = (bt >= lo) & (bt < lo + int(window_s * 1e9))
            if sel.sum() >= 5:
                cx.append(float(np.median(bt[sel])))
                cy.append(float(np.percentile(off[sel], lower_pct)))
        if len(cx) >= 3:
            slope, icept = np.polyfit(np.asarray(cx) - cx[0], cy, 1)
            out[a:b] = bt + np.round(icept + slope * (bt - cx[0])).astype(np.int64)
        else:
            out[a:b] = bt + int(round(np.percentile(off, lower_pct)))
    return out


# ============================================================================
#  ROTATIONS
# ============================================================================

def quat_to_rpy(q):
    """(N, 4) or (4,) quaternions -> roll, pitch, yaw arrays (ZYX)."""
    q = np.atleast_2d(np.asarray(q, float))
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_to_quat(roll, pitch, yaw):
    """ZYX Euler angles -> (x, y, z, w), scalars or arrays (then (N, 4))."""
    cr, sr = np.cos(np.asarray(roll) / 2.0), np.sin(np.asarray(roll) / 2.0)
    cp, sp = np.cos(np.asarray(pitch) / 2.0), np.sin(np.asarray(pitch) / 2.0)
    cy, sy = np.cos(np.asarray(yaw) / 2.0), np.sin(np.asarray(yaw) / 2.0)
    return np.stack([sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy,
                     cr * cp * cy + sr * sp * sy], axis=-1)


def euler_rates(roll, pitch, p, q, r):
    """Body rates (p, q, r) -> ZYX Euler-angle rates (droll, dpitch, dyaw)."""
    sph, cph = np.sin(roll), np.cos(roll)
    cth = np.cos(pitch)
    cth = np.where(np.abs(cth) < 1e-6, 1e-6, cth)
    tth = np.sin(pitch) / cth
    return (p + sph * tth * q + cph * tth * r,
            cph * q - sph * r,
            (sph * q + cph * r) / cth)


# ============================================================================
#  ATTITUDE TRACK
# ============================================================================

class AttitudeTrack:
    """Attitude as a function of bag time: cubic Hermite on the unwrapped
    Euler angles, tangents from the recorded body rates (finite differences
    when the source carries none); linear across a gap > MAX_HERMITE_GAP_S;
    held constant before the first and after the last sample."""

    def __init__(self, t_ns, quats, body_rates=None, max_gap_s=MAX_HERMITE_GAP_S):
        t = np.asarray(t_ns, np.int64)
        order = np.argsort(t, kind='stable')
        t = t[order]
        keep = np.concatenate([[True], np.diff(t) > 0])   # one sample per instant
        idx = order[keep]
        self.t = t[keep]
        roll, pitch, yaw = quat_to_rpy(np.asarray(quats, float)[idx])
        self.rpy = np.stack([np.unwrap(roll), pitch, np.unwrap(yaw)], axis=1)
        ts = self.t * 1e-9
        if body_rates is not None and len(self.t) > 0:
            w = np.asarray(body_rates, float)[idx]
            self.rpy_dot = np.stack(euler_rates(roll, pitch, w[:, 0], w[:, 1], w[:, 2]), axis=1)
            self.tangents = 'rates'
        elif len(self.t) >= 2:
            self.rpy_dot = np.gradient(self.rpy, ts, axis=0)
            self.tangents = 'differences'
        else:
            self.rpy_dot = np.zeros_like(self.rpy)
            self.tangents = 'none'
        self.max_gap_ns = int(max_gap_s * 1e9)

    def __len__(self):
        return len(self.t)

    @property
    def rate_hz(self):
        """Median sample rate (Hz), 0 with fewer than two samples."""
        if len(self.t) < 2:
            return 0.0
        return float(1e9 / max(1.0, np.median(np.diff(self.t))))

    def rpy_at(self, t_ns):
        """(roll, pitch, yaw) at bag time t_ns (yaw unwrapped)."""
        i = int(np.searchsorted(self.t, t_ns))
        if i <= 0:
            return self.rpy[0].copy()
        if i >= len(self.t):
            return self.rpy[-1].copy()
        a, b = i - 1, i
        h_ns = int(self.t[b] - self.t[a])
        s = (t_ns - self.t[a]) / h_ns
        y0, y1 = self.rpy[a], self.rpy[b]
        if h_ns > self.max_gap_ns:
            return y0 + s * (y1 - y0)
        h = h_ns * 1e-9
        s2, s3 = s * s, s * s * s
        return ((2 * s3 - 3 * s2 + 1) * y0 + (s3 - 2 * s2 + s) * h * self.rpy_dot[a]
                + (-2 * s3 + 3 * s2) * y1 + (s3 - s2) * h * self.rpy_dot[b])

    def at(self, t_ns):
        """The quaternion (x, y, z, w) at bag time t_ns."""
        r, p, y = self.rpy_at(t_ns)
        return tuple(float(v) for v in rpy_to_quat(r, p, y))


# ============================================================================
#  HEAVE
# ============================================================================

def detrended(t_s, z, window_s=HEAVE_WINDOW_S):
    """z minus its centred moving mean over window_s (unclipped)."""
    z = np.asarray(z, float)
    if len(z) == 0:
        return z
    t_s = np.asarray(t_s, float)
    c = np.concatenate([[0.0], np.cumsum(z)])
    lo = np.searchsorted(t_s, t_s - window_s / 2.0, side='left')
    hi = np.searchsorted(t_s, t_s + window_s / 2.0, side='right')
    return z - (c[hi] - c[lo]) / np.maximum(hi - lo, 1)


def heave_gate(t_s, z, window_s=HEAVE_WINDOW_S, clip=HEAVE_CLIP_M,
               max_std=HEAVE_MAX_STD_M, max_clip_fraction=HEAVE_MAX_CLIP_FRACTION):
    """(heave or None, stats). The detrended, clipped EKF z when it looks like
    wave motion (std <= max_std and at most max_clip_fraction of samples
    clipped); None when it is altitude-estimate noise and the waterline must
    be pinned. stats = {std_m, clipped_fraction}."""
    d = detrended(t_s, z, window_s)
    if len(d) == 0:
        return None, {'std_m': 0.0, 'clipped_fraction': 0.0}
    stats = {'std_m': float(d.std()),
             'clipped_fraction': float(np.mean(np.abs(d) > clip))}
    if stats['std_m'] > max_std or stats['clipped_fraction'] > max_clip_fraction:
        return None, stats
    return np.clip(d, -clip, clip), stats


def parse_heave(value):
    """The `heave` parameter: 'auto' (default), or a boolean / its string."""
    s = str(value).strip().lower()
    if s in ('', 'auto'):
        return 'auto'
    if s in ('true', '1', 'yes', 'on'):
        return True
    if s in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f"heave must be auto, true or false, not {value!r}")
