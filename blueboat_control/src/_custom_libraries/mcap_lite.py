#!/usr/bin/env python3
r"""
A minimal, dependency-free reader for this project's run bags (MCAP) and the CDR
of the few message types the pinger tools need.

ROS-FREE BY CONSTRUCTION (stdlib + numpy). Moved here on 2026-11-24 from
`log_reviewer/reviewer/pinger_bag.py`, which now re-exports it, so the log
reviewer, the pinger_filter_replay CLI and poslog_report share ONE reader.

Why hand-rolled: neither `mcap` nor `rosbag2_py` is available to every
interpreter that reads bags here, and run bags have UNCOMPRESSED chunks
(`compression_format: ""`). Compressed chunks are read only if `zstandard` /
`lz4` happen to be installed; otherwise the reader says so.

Times: only the MCAP `log_time` (the recorder's RECEIVE time, epoch ns) is
used. Header stamps of mavros-derived messages read 0 on the real launch.
"""

import mmap
import os
import struct

import numpy as np


class McapError(Exception):
    """The bag cannot be read (the message says why)."""


def mcap_files(bag):
    """The .mcap files of a bag folder (or [bag] for a file), sorted.

    os.listdir, not glob: a bag filed from staging keeps a hidden
    `.run-bag-<pid>-<k>_0.mcap` name, which a `*.mcap` glob skips.
    """
    if os.path.isfile(bag):
        return [bag]
    names = sorted(n for n in os.listdir(bag) if n.endswith(".mcap"))
    return [os.path.join(bag, n) for n in names]


def _string(buf, pos):
    n = struct.unpack_from("<I", buf, pos)[0]
    return bytes(buf[pos + 4:pos + 4 + n]).decode("utf-8", "replace"), pos + 4 + n


def _decompress(name, blob):
    if name == "zstd":
        try:
            import zstandard
        except ImportError as exc:
            raise McapError("the bag's chunks are zstd-compressed and "
                            "'zstandard' is not installed") from exc
        return zstandard.ZstdDecompressor().decompress(blob)
    if name == "lz4":
        try:
            import lz4.frame
        except ImportError as exc:
            raise McapError("the bag's chunks are lz4-compressed and "
                            "'lz4' is not installed") from exc
        return lz4.frame.decompress(blob)
    raise McapError("unknown MCAP chunk compression %r" % name)


def _records(buf, pos, end):
    while pos + 9 <= end:
        op = buf[pos]
        length = struct.unpack_from("<Q", buf, pos + 1)[0]
        yield op, pos + 9, pos + 9 + length
        pos += 9 + length


def read_topics(path, topics):
    """{topic: [(log_time_ns, payload bytes), ...]} and the file's (first, last)
    message time in ns. Only the wanted topics are copied out."""
    wanted = set(topics)
    out = {topic: [] for topic in topics}
    channels = {}
    span = [None, None]

    def visit(buf, start, end):
        for op, a, b in _records(buf, start, end):
            if op == 0x04:                                   # Channel
                cid = struct.unpack_from("<H", buf, a)[0]
                channels[cid] = _string(buf, a + 4)[0]
            elif op == 0x05:                                 # Message
                cid, _seq, log_time = struct.unpack_from("<HIQ", buf, a)
                topic = channels.get(cid)
                if topic in wanted:
                    out[topic].append((log_time, bytes(buf[a + 22:b])))

    with open(path, "rb") as handle:
        with mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ) as buf:
            if bytes(buf[:8]) != b"\x89MCAP0\r\n":
                raise McapError("%s is not an MCAP file" % os.path.basename(path))
            for op, a, b in _records(buf, 8, len(buf) - 8):
                if op == 0x06:                               # Chunk
                    first, last = struct.unpack_from("<QQ", buf, a)
                    span[0] = first if span[0] is None else min(span[0], first)
                    span[1] = last if span[1] is None else max(span[1], last)
                    compression, p = _string(buf, a + 28)
                    records_len = struct.unpack_from("<Q", buf, p)[0]
                    body = bytes(buf[p + 8:p + 8 + records_len])
                    if compression:
                        body = _decompress(compression, body)
                    visit(body, 0, len(body))
                elif op == 0x0F:                             # DataEnd
                    break
                else:
                    visit(buf, a - 9, b)                     # loose records
    return out, tuple(span)


def read_bag(bag, topics):
    """All files of a bag merged: {topic: [(log_time_ns, payload)] sorted by time}
    and the bag's (first, last) time in ns (None if empty)."""
    files = mcap_files(bag)
    if not files:
        raise McapError("no .mcap file inside %s" % bag)
    out = {t: [] for t in topics}
    first = last = None
    for path in files:
        part, span = read_topics(path, topics)
        for t in topics:
            out[t].extend(part[t])
        if span[0] is not None:
            first = span[0] if first is None else min(first, span[0])
            last = span[1] if last is None else max(last, span[1])
    for t in topics:
        out[t].sort(key=lambda r: r[0])
    return out, (first, last)


# ---------------------------------------------------------------------------
# CDR (little endian, encapsulation header = 4 bytes; alignment is relative to
# the byte after it)
# ---------------------------------------------------------------------------

def _align(pos, a):
    r = (pos - 4) % a
    return pos if r == 0 else pos + a - r


def _skip_string(p, pos):
    pos = _align(pos, 4)
    n = struct.unpack_from("<I", p, pos)[0]
    return pos + 4 + n


def _check(payload):
    if len(payload) < 8 or payload[1] != 1:
        raise McapError("unexpected CDR encoding (not little-endian CDR)")


def decode_float32_multiarray(payload):
    """The `data` of a std_msgs/Float32MultiArray in little-endian CDR."""
    if len(payload) < 16 or payload[1] != 1:
        raise McapError("unexpected CDR encoding in a Float32MultiArray")
    pos = 4
    dims = struct.unpack_from("<I", payload, pos)[0]
    pos += 4
    for _ in range(dims):                       # MultiArrayDimension: label, size, stride
        n = struct.unpack_from("<I", payload, pos)[0]
        pos = (pos + 4 + n + 3) & ~3
        pos += 8
    pos = (pos + 3) & ~3
    pos += 4                                    # layout.data_offset
    count = struct.unpack_from("<I", payload, pos)[0]
    pos += 4
    return np.frombuffer(payload, "<f4", count, pos).astype(float)


def decode_string(payload):
    """The `data` of a std_msgs/String."""
    _check(payload)
    n = struct.unpack_from("<I", payload, 4)[0]
    return bytes(payload[8:8 + max(0, n - 1)]).decode("utf-8", "replace")


def _header_end(p):
    """Position after std_msgs/Header (stamp sec/nanosec + frame_id)."""
    return _skip_string(p, 12)


def decode_odometry(payload):
    """nav_msgs/Odometry -> dict(stamp, x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz).
    The twist is as published (body frame for MAVROS / /blueboat/odom)."""
    _check(payload)
    sec, nsec = struct.unpack_from("<iI", payload, 4)
    pos = _header_end(payload)
    pos = _skip_string(payload, pos)            # child_frame_id
    pos = _align(pos, 8)
    x, y, z, qx, qy, qz, qw = struct.unpack_from("<7d", payload, pos)
    pos += 56 + 36 * 8
    vx, vy, vz, wx, wy, wz = struct.unpack_from("<6d", payload, pos)
    return {"stamp": sec + nsec * 1e-9, "x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            "vx": vx, "vy": vy, "vz": vz, "wx": wx, "wy": wy, "wz": wz}


def decode_imu(payload):
    """sensor_msgs/Imu -> dict(stamp, orientation (x,y,z,w), angular_velocity (x,y,z),
    linear_acceleration (x,y,z))."""
    _check(payload)
    sec, nsec = struct.unpack_from("<iI", payload, 4)
    pos = _align(_header_end(payload), 8)
    q = struct.unpack_from("<4d", payload, pos)
    pos += 32 + 72
    w = struct.unpack_from("<3d", payload, pos)
    pos += 24 + 72
    a = struct.unpack_from("<3d", payload, pos)
    return {"stamp": sec + nsec * 1e-9, "orientation": q, "angular_velocity": w,
            "linear_acceleration": a}


def yaw_of(qx, qy, qz, qw):
    """ENU yaw of a quaternion (the same formula custom_functions.quaternion_to_yaw uses)."""
    import math
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
