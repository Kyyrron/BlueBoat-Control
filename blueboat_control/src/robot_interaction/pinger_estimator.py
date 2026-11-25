#!/usr/bin/env python3
"""
pinger_estimator (node `pinger_estimator`): the USBL pinger, live. The one
publisher of /blueboat/pinger_coordinates and /blueboat/pinger/*; the filter
itself is pinger_filter.py, its parameters config/pinger_filter.yaml.

Inputs (timing = receive time on this node's clock; mavros stamps read 0 live)
  /blueboat/odom        boat pose (local ENU) + body twist
  /mavros/imu/data      yaw rate (legacy dead reckoning)
  /uw_gps/fix           JSON fixes from uwgps_log: the filter's input
  /uw_gps_data          the legacy 19-value packet: the legacy pinger
  /blueboat/run_command a `configure` carrying `pinger_filter` {preset, params}
                        and/or `pinger_source` (the station's filter dialog)
Outputs
  /blueboat/pinger_coordinates  [x, y, z] body frame at odom rate: the filtered
      vector b = R(psi)^T (p_hat - p_boat) (zeros until the filter starts) with
      pinger_source 'filter', the v1.1 vector with 'legacy'.
  /blueboat/pinger/legacy_body  the legacy vector, always.
  /blueboat/pinger/estimate     JSON, at each fix and every second.
  /blueboat/pinger/fix          JSON, one record per processed fix (depth 50).
  /blueboat/pinger/filter_config JSON, latched: the set in force, its hash, the
      source and the `configure` seq it answers.

Tuning, live: `pinger_filter_yaml` names a tuning file (created with every
parameter listed if missing); each save is applied within a second, and so is
a station `configure`, at any time. The estimate is kept across a change.
"""

import json
import math
import os
import sys

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String

# Flat beside this file in lib/blueboat_control (installed), or the source tree.
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (_HERE, os.path.join(_HERE, "..", "_custom_libraries")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import pinger_params as pp                          # noqa: E402
from pinger_filter import LegacyPinger, PingerFilter, fix_from_json, status_of_age   # noqa: E402

SOURCES = ("filter", "legacy")
HEARTBEAT_S = 1.0       # the estimate is re-sent this often between fixes (master_control's
                        # pinger_stale_timeout is 3 s)


def _finite(obj):
    """obj with every non-finite float replaced by None (strict JSON)."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _finite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_finite(v) for v in obj]
    return obj


def _string(obj):
    m = String()
    m.data = json.dumps(_finite(obj), default=float, allow_nan=False)
    return m


class PingerEstimator(Node):

    def __init__(self):
        super().__init__('pinger_estimator')
        self.source = self.declare_parameter('pinger_source', 'filter').value
        self.preset = self.declare_parameter('pinger_preset', 'slow_rov').value or pp.DEFAULT_PRESET
        self.tuning_file = os.path.expanduser(self.declare_parameter('pinger_filter_yaml', '').value)
        if self.source not in SOURCES:
            raise ValueError(f"pinger_source {self.source!r} is not one of {', '.join(SOURCES)}")
        if self.tuning_file and not os.path.exists(self.tuning_file):
            pp.write_tuning_template(self.tuning_file, pp.load_params(preset=self.preset))
            self.get_logger().info(f"Pinger tuning file created: {self.tuning_file}")
        self.params = pp.load_params(preset=self.preset, yaml_path=self.tuning_file or None)
        self.tuning_mtime = os.path.getmtime(self.tuning_file) if self.tuning_file else None

        self.filter = PingerFilter(self.params.values)
        self.legacy = LegacyPinger()
        self.last_pose = None           # (x, y) of the latest odom
        self.last_packet_t = None       # receive time of the last usable legacy packet

        fix_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.body_pub = self.create_publisher(Float32MultiArray, '/blueboat/pinger_coordinates', 10)
        self.legacy_pub = self.create_publisher(Float32MultiArray, '/blueboat/pinger/legacy_body', 10)
        self.estimate_pub = self.create_publisher(String, '/blueboat/pinger/estimate', 10)
        self.fix_pub = self.create_publisher(String, '/blueboat/pinger/fix', fix_qos)
        self.config_pub = self.create_publisher(String, '/blueboat/pinger/filter_config', latched)

        imu_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, imu_qos)
        self.create_subscription(Float32MultiArray, '/uw_gps_data', self.packet_callback, 10)
        self.create_subscription(String, '/uw_gps/fix', self.fix_callback, fix_qos)
        self.create_subscription(String, '/blueboat/run_command', self.run_command_callback, 10)
        self.create_timer(HEARTBEAT_S, self.heartbeat_callback)

        self.publish_config(True, "applied at start")
        self.get_logger().info(
            f"Pinger estimator: source={self.source}, preset={self.params.preset}, hash={self.params.hash}"
            + (f", tuning live from {self.tuning_file}" if self.tuning_file else ""))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _guard(self, what, fn, *args):
        """A malformed input must not kill the node (every pinger topic would go
        silent for the rest of the session): report it and drop it."""
        try:
            fn(*args)
        except Exception as exc:                        # noqa: BLE001 - reported
            self.get_logger().error(f"pinger_estimator: {what} dropped: {type(exc).__name__}: {exc}",
                                    throttle_duration_sec=5.0)

    # ---------------------------------------------------------------- inputs
    def odom_callback(self, msg: Odometry):
        self._guard('odometry', self._on_odom, msg)

    def _on_odom(self, msg):
        t = self._now()
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.filter.add_pose(t, p.x, p.y, yaw)
        self.last_pose = (p.x, p.y)
        legacy = self.legacy.on_odom(t, (v.x, v.y, v.z))
        if legacy is not None:
            self._publish_vector(self.legacy_pub, legacy)
        if self.source == 'legacy':
            body = legacy
        else:
            body = self.filter.body_at(p.x, p.y, yaw) or [0.0, 0.0, 0.0]
        if body is not None:
            self._publish_vector(self.body_pub, body)

    def imu_callback(self, msg: Imu):
        self.legacy.on_imu(msg.angular_velocity.z)

    def packet_callback(self, msg: Float32MultiArray):
        data = [float(v) for v in msg.data]
        if len(data) < 19 or not all(math.isfinite(v) for v in data[16:19]):
            return                  # a non-finite packet would turn the legacy vector into NaN
        self.legacy.on_ugps(data)
        if any(v != 0.0 for v in data[16:19]):
            self.last_packet_t = self._now()

    def fix_callback(self, msg: String):
        self._guard('/uw_gps/fix message', self._on_fix, msg)

    def _on_fix(self, msg):
        t = self._now()
        fix = fix_from_json(t, msg.data, self.params['source'])
        if fix is None:
            return
        res = self.filter.add_fix(fix)
        res["schema"] = "blueboat_pinger_fix/1"
        self.fix_pub.publish(_string(res))
        if res["decision"] != "duplicate":
            self._publish_estimate(t)
        if res["event"]:
            self.get_logger().info(f"Pinger {res['event']} at ({res['posterior']['x']:.1f}, "
                                   f"{res['posterior']['y']:.1f}) - {res['reason']}")

    def heartbeat_callback(self):
        self._guard('heartbeat', self._publish_estimate, self._now())
        if self.tuning_file:
            self._guard('tuning file', self._reload_tuning_file)

    # --------------------------------------------------------------- tuning
    def _reload_tuning_file(self):
        try:
            mtime = os.path.getmtime(self.tuning_file)
        except OSError:
            return                                      # being replaced by an editor: next second
        if mtime == self.tuning_mtime:
            return
        self.tuning_mtime = mtime
        try:
            params = pp.load_params(preset=self.preset, yaml_path=self.tuning_file)
        except (pp.ParamError, OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f"Pinger tuning file refused, previous set kept: {exc}")
            return
        self._apply(params, self.source, None, f"tuning file {os.path.basename(self.tuning_file)}")

    def run_command_callback(self, msg: String):
        self._guard('configure', self._on_run_command, msg)

    def _on_run_command(self, msg):
        """The `pinger_filter` {preset, params} / `pinger_source` block of a
        station `configure` (master_control handles the rest of it)."""
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(cmd, dict) or cmd.get('cmd') != 'configure':
            return
        block, source = cmd.get('pinger_filter'), cmd.get('pinger_source')
        if block is None and source is None:
            return                                      # nothing for this node
        seq = cmd.get('seq')
        try:
            source = self.source if source is None else str(source)
            if source not in SOURCES:
                raise pp.ParamError(f"pinger_source {source!r} is not one of {', '.join(SOURCES)}")
            params = self.params
            if block is not None:
                if not isinstance(block, dict) or not isinstance(block.get('params') or {}, dict):
                    raise pp.ParamError("pinger_filter must be {preset, params: {...}}")
                params = pp.load_params(preset=str(block.get('preset') or pp.DEFAULT_PRESET),
                                        overrides=block.get('params') or {})
        except (pp.ParamError, ValueError, OverflowError, TypeError) as exc:
            self.publish_config(False, f"refused: {exc}", seq)
            self.get_logger().error(f"Pinger filter configure refused: {exc}")
            return
        self._apply(params, source, seq, "station configure")

    def _apply(self, params, source, seq, origin):
        """Make `params` / `source` the set in force, live; echo it."""
        changed = params.hash != self.params.hash or source != self.source
        self.params, self.source = params, source
        self.filter.set_params(params.values)
        detail = ("applied" if changed else "unchanged") + f" (hash {params.hash}, source {source})"
        self.publish_config(True, detail, seq)
        if changed:
            self._publish_estimate(self._now())         # no stale status left cached anywhere
            self.get_logger().info(f"Pinger filter {detail}, preset {params.preset}, from {origin}.")

    # ---------------------------------------------------------------- outputs
    @staticmethod
    def _publish_vector(pub, vec):
        m = Float32MultiArray()
        m.data = [float(v) for v in vec]
        pub.publish(m)

    def _legacy_status(self, t):
        """The legacy input's freshness, on the filter's coast / stale times."""
        if self.last_packet_t is None:
            return "no_data"
        return status_of_age(t - self.last_packet_t, self.params)

    def _publish_estimate(self, t):
        out = self.filter.estimate(t)
        out["schema"] = "blueboat_pinger_estimate/1"
        out["source"] = self.source
        # What /blueboat/pinger_coordinates carries is this fresh (master_control's follow gate).
        out["source_status"] = out["status"] if self.source == "filter" else self._legacy_status(t)
        out["range_m"] = None
        if self.last_pose is not None and out["x"] is not None:
            out["range_m"] = math.hypot(out["x"] - self.last_pose[0], out["y"] - self.last_pose[1])
        self.estimate_pub.publish(_string(out))

    def publish_config(self, ok, detail, seq=None):
        self.config_pub.publish(_string({
            "schema": "blueboat_pinger_filter_config/1", "preset": self.params.preset,
            "params": self.params.values, "hash": self.params.hash, "pinger_source": self.source,
            "seq": seq, "ok": bool(ok), "detail": str(detail), "t": self._now()}))


def main():
    rclpy.init()
    node = PingerEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
