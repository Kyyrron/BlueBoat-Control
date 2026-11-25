#!/usr/bin/env python3
"""
pinger_estimator (node `pinger_estimator`): the whole USBL pinger pipeline.

Since 2026-11-24 this node -- not robot_interface -- is the ONE publisher of
/blueboat/pinger_coordinates, and of every /blueboat/pinger/* topic. All the
computation is in the ROS-free `pinger_estimator_core` (the same code the
offline `pinger_filter_replay` CLI runs); this file only wires topics to it.

Inputs (receive time on this node's clock is the timing truth: header stamps of
mavros-derived messages read 0 on the real launch)
  /blueboat/odom        nav_msgs/Odometry   pose (local ENU) + body twist
  /mavros/imu/data      sensor_msgs/Imu     yaw rate, for the legacy dead reckoning
  /uw_gps/fix           std_msgs/String     JSON fixes (the filter's input)
  /uw_gps_data          Float32MultiArray   legacy packet (legacy reseed; filter
                                            fallback when /uw_gps/fix is silent)
  /blueboat/run_command std_msgs/String     `configure` with an additive
                                            `pinger_filter` / `pinger_source` block
  /blueboat/run_state   std_msgs/String     whether a run is active
Outputs
  /blueboat/pinger_coordinates  Float32MultiArray [x, y, z] body frame, at odom
      rate: the FILTERED vector b = R(psi)^T (p_hat - p_boat) (zeros until the
      filter initialises) with pinger_source 'filter' (default); exactly the v1.1
      robot_interface vector with 'legacy'.
  /blueboat/pinger/legacy_body  Float32MultiArray, the legacy vector, always.
  /blueboat/pinger/estimate     String JSON blueboat_pinger_estimate/1, at each
      processed fix and at heartbeat_hz.
  /blueboat/pinger/fix          String JSON blueboat_pinger_fix/1, every processed
      fix (reliable, depth 50).
  /blueboat/pinger/filter_config String JSON, TRANSIENT_LOCAL depth 1: the applied
      parameter set, its hash, the source, the configure seq it answers.

Parameters: pinger_source (filter|legacy), pinger_preset (slow_rov), pinger_filter_yaml
(''), allow_live_retune (false), clock_source (node|replay), snapshot_period_s (10).
A `configure` during a run is refused unless allow_live_retune; the filter restarts
only when the parameter set actually changes; every applied change is logged and
echoed.

Filter snapshots (`decision: state` on /blueboat/pinger/fix) at every run start, after
every parameter change and every snapshot_period_s: a run's per-fix log starts at
one, so it re-runs the live filter bit for bit.

Clock. Live: this node's clock (wall, use_sim_time false). In a bag replay
(clock_source:=replay) the bag's own timeline, rebuilt from bag_replay's
/replay/state (position, rate, pause): every duration of the filter (latency,
coast / stale, windows) is then bag time, whatever the replay rate or Gazebo's
real-time factor.
"""

import json
import math
import os
import sys
import time

import rclpy
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

import mcap_lite                                    # noqa: E402  (yaw_of: one formula online and offline)
import pinger_params as pp                          # noqa: E402
from pinger_estimator_core import SOURCES, EstimatorCore    # noqa: E402


def _finite(obj):
    """obj with every non-finite float replaced by None (strict JSON)."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _finite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_finite(v) for v in obj]
    return obj


def _json(obj):
    return json.dumps(_finite(obj), default=float, allow_nan=False)


class PingerEstimator(Node):

    def __init__(self):
        super().__init__('pinger_estimator')
        source = self.declare_parameter('pinger_source', 'filter').value
        self.preset = self.declare_parameter('pinger_preset', 'slow_rov').value
        self.filter_yaml = self.declare_parameter('pinger_filter_yaml', '').value
        self.allow_live_retune = bool(self.declare_parameter('allow_live_retune', False).value)
        self.clock_source = self.declare_parameter('clock_source', 'node').value
        snapshot_period = float(self.declare_parameter('snapshot_period_s', 10.0).value)
        if self.clock_source not in ('node', 'replay'):
            raise ValueError(f"clock_source {self.clock_source!r} is not node or replay")

        params = pp.load_params(preset=self.preset or pp.DEFAULT_PRESET,
                                yaml_path=self.filter_yaml or None)
        # The live node never smooths: no forward-pass history (hours of fixes).
        # The core validates `source` (and owns it from here: configure).
        self.core = EstimatorCore(params, source, keep_steps=False)
        self.run_active = False
        self.run_number = None
        self.replay_clock = None        # (bag t, monotonic at it, rate, paused) from /replay/state
        self.last_t = -math.inf

        # ---- publishers (absolute names: one contract, every launch) ----
        fix_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.body_pub = self.create_publisher(Float32MultiArray, '/blueboat/pinger_coordinates', 10)
        self.legacy_pub = self.create_publisher(Float32MultiArray, '/blueboat/pinger/legacy_body', 10)
        self.estimate_pub = self.create_publisher(String, '/blueboat/pinger/estimate', 10)
        self.fix_pub = self.create_publisher(String, '/blueboat/pinger/fix', fix_qos)
        self.config_pub = self.create_publisher(String, '/blueboat/pinger/filter_config', latched)

        # ---- subscribers ----
        imu_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, imu_qos)
        self.create_subscription(Float32MultiArray, '/uw_gps_data', self.packet_callback, 10)
        self.create_subscription(String, '/uw_gps/fix', self.fix_callback, fix_qos)
        self.create_subscription(String, '/blueboat/run_command', self.run_command_callback, 10)
        self.create_subscription(String, '/blueboat/run_state', self.run_state_callback, 10)
        if self.clock_source == 'replay':
            replay_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, '/replay/state', self.replay_state_callback, replay_qos)

        self.heartbeat = self.create_timer(1.0 / self.core.values['heartbeat_hz'], self.heartbeat_callback)
        self.snapshot_timer = self.create_timer(max(1.0, snapshot_period), self.snapshot_callback)
        self.publish_config(ok=True, detail="applied at start")
        self.get_logger().info(
            f"Pinger estimator: source={source}, preset={params.preset}, "
            f"hash={params.hash}, latency={params.values['latency_s']} s (provisional until field "
            "calibration), publishing /blueboat/pinger_coordinates and /blueboat/pinger/*")

    # ----------------------------------------------------------------- time
    def _now(self):
        """Receive time: this node's clock, or the replayed bag's timeline.
        Never goes backwards (a /replay/state correction is absorbed)."""
        if self.clock_source == 'replay':
            if self.replay_clock is None:
                t = 0.0                     # bag time starts at 0 (before the first state)
            else:
                t0, mono0, rate, paused = self.replay_clock
                t = t0 if paused else t0 + (time.monotonic() - mono0) * rate
        else:
            t = self.get_clock().now().nanoseconds * 1e-9
        self.last_t = max(self.last_t, t)
        return self.last_t

    def replay_state_callback(self, msg: String):
        try:
            st = json.loads(msg.data)
            self.replay_clock = (float(st['t']), time.monotonic(), float(st.get('rate', 1.0)),
                                 bool(st.get('paused', False)))
        except (ValueError, KeyError, TypeError):
            pass

    # ---------------------------------------------------------------- inputs
    def _guard(self, what, fn, *args):
        """A malformed input must not kill the node (every pinger topic would
        go silent for the rest of the session): report it and drop it."""
        try:
            fn(*args)
        except Exception as exc:                        # noqa: BLE001 - reported
            self.get_logger().error(f"pinger_estimator: {what} dropped: {type(exc).__name__}: {exc}",
                                    throttle_duration_sec=5.0)

    def odom_callback(self, msg: Odometry):
        self._guard('odometry', self._on_odom, msg)

    def _on_odom(self, msg: Odometry):
        t = self._now()
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        out = self.core.on_odom(t, p.x, p.y, mcap_lite.yaw_of(q.x, q.y, q.z, q.w), v.x, v.y, v.z)
        if out["legacy_body"] is not None:
            self._publish_vector(self.legacy_pub, out["legacy_body"])
        if out["body"] is not None:
            self._publish_vector(self.body_pub, out["body"])

    def imu_callback(self, msg: Imu):
        self._guard('IMU', lambda: self.core.on_imu(self._now(), msg.angular_velocity.z))

    def packet_callback(self, msg: Float32MultiArray):
        def go():
            t = self._now()
            self._publish_results(t, self.core.on_packet(t, list(msg.data)))
        self._guard('/uw_gps_data packet', go)

    def fix_callback(self, msg: String):
        def go():
            t = self._now()
            self._publish_results(t, self.core.on_fix_json(t, msg.data))
        self._guard('/uw_gps/fix message', go)

    def heartbeat_callback(self):
        self._guard('heartbeat', lambda: self._publish_estimate(self._now()))

    def snapshot_callback(self):
        self._guard('snapshot', lambda: self._publish_snapshot("periodic"))

    def _publish_snapshot(self, reason):
        m = String()
        m.data = _json(self.core.snapshot_record(self._now(), reason, self.run_number))
        self.fix_pub.publish(m)

    # ------------------------------------------------------------- outputs
    @staticmethod
    def _publish_vector(pub, vec):
        m = Float32MultiArray()
        m.data = [float(v) for v in vec]
        pub.publish(m)

    def _publish_results(self, t, results):
        for res in results:
            m = String()
            m.data = _json(res)
            self.fix_pub.publish(m)
            if res["decision"] != "duplicate":
                self._publish_estimate(t)
            if res.get("event"):
                self.get_logger().info(f"Pinger {res['event']} at ({res['posterior']['x']:.1f}, "
                                       f"{res['posterior']['y']:.1f}) - {res['reason']}")

    def _publish_estimate(self, t):
        m = String()
        m.data = _json(self.core.estimate(t))
        self.estimate_pub.publish(m)

    def publish_config(self, ok, detail, seq=None):
        m = String()
        payload = self.core.config_payload(seq=seq, ok=ok, detail=detail)
        payload["allow_live_retune"] = self.allow_live_retune
        payload["t"] = self._now()
        m.data = _json(payload)
        self.config_pub.publish(m)

    # ------------------------------------------------------------- session
    def run_state_callback(self, msg: String):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(state, dict):
            running = state.get('state') == 'running'
            run = state.get('run')
            if running and run != self.run_number:
                # A new run: a snapshot its per-fix log starts at.
                self.run_number = run
                self._guard('run-start snapshot', lambda: self._publish_snapshot(f"run {run} start"))
            self.run_active = running

    def run_command_callback(self, msg: String):
        self._guard('configure', self._on_run_command, msg)

    def _on_run_command(self, msg: String):
        """Apply the additive `pinger_filter` {preset, params} / `pinger_source`
        block of a `configure` (master_control handles the rest of it)."""
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(cmd, dict) or cmd.get('cmd') != 'configure':
            return
        block = cmd.get('pinger_filter')
        source = cmd.get('pinger_source')
        if block is None and source is None:
            return                              # an older station: nothing for this node
        seq = cmd.get('seq')
        if self.run_active and not self.allow_live_retune:
            self.publish_config(False, "a run is active: filter parameters are applied between runs "
                                       "(launch with allow_live_retune:=true to change them live)", seq)
            self.get_logger().warning("Pinger filter configure refused: a run is active.")
            return
        try:
            new_source = self.core.pinger_source if source is None else str(source)
            if new_source not in SOURCES:
                raise pp.ParamError(f"pinger_source {new_source!r} is not one of {', '.join(SOURCES)}")
            params = self.core.params
            if block is not None:
                if not isinstance(block, dict):
                    raise pp.ParamError("pinger_filter must be an object {preset, params}")
                preset = str(block.get('preset') or pp.DEFAULT_PRESET)
                overrides = block.get('params') or {}
                if not isinstance(overrides, dict):
                    raise pp.ParamError("pinger_filter.params must be an object")
                params = pp.load_params(preset=preset, overrides=overrides)
        except (pp.ParamError, ValueError, OverflowError, TypeError) as exc:
            self.publish_config(False, f"refused: {exc}", seq)
            self.get_logger().error(f"Pinger filter configure refused: {exc}")
            return
        old_hash, old_source = self.core.params.hash, self.core.pinger_source
        reset = params.hash != old_hash
        changed = reset or old_source != new_source
        if reset:
            # Only a CHANGED set restarts the filter (the station re-sends the
            # same block with every configure).
            self.core.set_params(params)
        self.core.pinger_source = new_source
        if self.heartbeat.timer_period_ns != int(1e9 / self.core.values['heartbeat_hz']):
            self.heartbeat.cancel()
            self.heartbeat = self.create_timer(1.0 / self.core.values['heartbeat_hz'], self.heartbeat_callback)
        detail = ("applied" if changed else "unchanged") + f" (hash {params.hash}, source {new_source}" + \
                 (", live retune" if self.run_active else "") + ")"
        self.publish_config(True, detail, seq)
        if changed:
            # No stale 'tracking' left cached anywhere, and a snapshot the next
            # per-fix log can start at.
            self._publish_estimate(self._now())
            self._publish_snapshot("parameters changed" if reset else "source changed")
        self.get_logger().info(f"Pinger filter configure: {detail}, preset {params.preset}, "
                               f"was hash {old_hash} source {old_source}.")


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
