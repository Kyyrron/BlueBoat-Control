#!/usr/bin/env python3
"""
Water Linked Underwater GPS poller (node `underwater_gps_logger`).

Publishes
  /uw_gps/fix   std_msgs/String, JSON `blueboat_uwgps_fix/1`, reliable depth 50.
                One message per poll of the raw acoustic position (raw_rate_hz,
                default 4 Hz): every field of every endpoint polled in that
                cycle, each with its request/response time on the ROS clock,
                HTTP status and error. Full cycles (full_rate_hz, default 2 Hz)
                also poll filtered and global (and antenna with --antenna).
                The pinger_estimator's input.
  /uw_gps_data  std_msgs/Float32MultiArray, 19 values -- the LEGACY packet,
                unchanged in layout and meaning and published on the same 2 Hz
                cycle as before (date x7 from the laptop clock at cycle start,
                raw xyz, antenna xyz, lat, lon, depth = raw z, filtered xyz; a
                failed request leaves zeros). Kept byte-compatible for the legacy
                pinger, the CSV's aco/filaco columns and old tools.

Every request has a (connect, read) timeout and uses one keep-alive session:
before 2026-11-24 a request had no timeout, and one hang stalled all pinger data
for 103 s.
"""

import argparse
import datetime
import json
import sys

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray, String

SCHEMA_FIX = "blueboat_uwgps_fix/1"
ENDPOINTS = {
    "raw": "/api/v1/position/acoustic/raw",
    "filtered": "/api/v1/position/acoustic/filtered",
    "antenna": "/api/v1/config/antenna",
    "global": "/api/v1/position/global",
}


class uw_gps_logger(Node):

    def __init__(self):
        super().__init__('underwater_gps_logger')

        self.declare_parameter('verbose', False)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.raw_rate_hz = float(self.declare_parameter('raw_rate_hz', 4.0).value)
        self.full_rate_hz = float(self.declare_parameter('full_rate_hz', 2.0).value)
        self.connect_timeout_s = float(self.declare_parameter('connect_timeout_s', 0.25).value)
        self.read_timeout_s = float(self.declare_parameter('read_timeout_s', 0.4).value)
        if self.raw_rate_hz <= 0.0 or self.full_rate_hz <= 0.0:
            raise ValueError("raw_rate_hz and full_rate_hz must be > 0")
        # A full cycle every `ratio` raw polls (2 at 4 / 2 Hz).
        self.ratio = max(1, int(round(self.raw_rate_hz / self.full_rate_hz)))

        self.parser = argparse.ArgumentParser(description=__doc__)
        self.parser.add_argument(
            "-u",
            "--url",
            help = "Base URL to use",
            type = str,
            default = "http://192.168.2.94")
        self.parser.add_argument(
            "-a",
            "--antenna",
            action = "store_true",
            help = (
                "Use mid-point of base of antenna as origin for the acoustic " +
                "position. Default origin is the point at sea level directly " +
                "below/above that which the positions of the receivers/antenna " +
                "are defined with respect to"))
        self.args = self.parser.parse_args(remove_ros_args(sys.argv)[1:])
        self.base_url = self.args.url

        self.session = requests.Session()
        fix_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.uw_data_pub = self.create_publisher(Float32MultiArray, '/uw_gps_data', 10)
        self.fix_pub = self.create_publisher(String, '/uw_gps/fix', fix_qos)
        self.seq = 0
        self.tick = 0
        self.timer = self.create_timer(1.0 / self.raw_rate_hz, self.timer_callback)

        self.get_logger().info(f'Using base_url: {self.base_url}')
        self.get_logger().info(
            f"Start Logging: raw fix at {self.raw_rate_hz:g} Hz, full cycle (and /uw_gps_data) "
            f"every {self.ratio} polls, timeouts {self.connect_timeout_s:g}/{self.read_timeout_s:g} s")

    # ------------------------------------------------------------------ HTTP
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _request(self, name, path):
        """One GET -> {t_req, t_resp, ok, status, error, data}. Never raises."""
        url = f"{self.base_url}{path}"
        rec = {"t_req": self._now(), "t_resp": None, "ok": False, "status": None,
               "error": None, "data": None}
        try:
            r = self.session.get(url, timeout=(self.connect_timeout_s, self.read_timeout_s))
            rec["status"] = r.status_code
            if r.status_code == requests.codes.ok:
                rec["data"] = r.json()
                rec["ok"] = True
            else:
                rec["error"] = f"HTTP {r.status_code}"
        except (requests.exceptions.RequestException, ValueError) as exc:
            rec["error"] = f"{type(exc).__name__}: {exc}"[:300]
        rec["t_resp"] = self._now()
        if not rec["ok"] and self.verbose:
            self.get_logger().info(f'{name}: {rec["error"]}')
        return rec

    # ------------------------------------------------------------------ cycle
    def timer_callback(self):
        if not rclpy.ok():
            return                                  # shutting down: nothing to publish to
        full = (self.tick % self.ratio) == 0
        self.tick += 1
        timestamp = datetime.datetime.now()          # the legacy packet's date fields
        t_start = self._now()

        names = ["raw"]
        if full:
            names += ["filtered"] + (["antenna"] if self.args.antenna else []) + ["global"]
        recs = {n: self._request(n, ENDPOINTS[n]) for n in names}

        self.seq += 1
        msg = String()
        msg.data = json.dumps({
            "schema": SCHEMA_FIX, "seq": self.seq, "cycle": "full" if full else "raw",
            "t_start": t_start, "t_end": self._now(),
            "wall_start": timestamp.timestamp(), "base_url": self.base_url,
            "endpoints": recs})
        self.fix_pub.publish(msg)

        if full:
            self.publish_legacy(timestamp, recs)

    def publish_legacy(self, timestamp, recs):
        """The /uw_gps_data packet exactly as v1.1 built it (zeros for a failed
        request; depth = raw z, set only when truthy)."""
        df_tmp = [0]*19

        df_tmp[0] = timestamp.year
        df_tmp[1] = timestamp.month
        df_tmp[2] = timestamp.day
        df_tmp[3] = timestamp.hour
        df_tmp[4] = timestamp.minute
        df_tmp[5] = timestamp.second
        df_tmp[6] = timestamp.microsecond

        acoustic_position = recs["raw"]["data"]
        acoustic_position_filtered = recs["filtered"]["data"]
        antenna_position = recs["antenna"]["data"] if "antenna" in recs else None
        global_position = recs["global"]["data"]
        depth = None

        try:
            if acoustic_position:
                df_tmp[7] = acoustic_position["x"]
                df_tmp[8] = acoustic_position["y"]
                df_tmp[9] = acoustic_position["z"]
                depth = acoustic_position["z"]
            elif self.verbose:
                self.get_logger().info('no acoustic position')

            if acoustic_position_filtered:
                df_tmp[16] = acoustic_position_filtered["x"]
                df_tmp[17] = acoustic_position_filtered["y"]
                df_tmp[18] = acoustic_position_filtered["z"]
            elif self.verbose:
                self.get_logger().info('no filtered acoustic position')

            if antenna_position:
                df_tmp[10] = antenna_position["x"]
                df_tmp[11] = antenna_position["y"]
                df_tmp[12] = antenna_position["z"]

            if global_position:
                df_tmp[13] = global_position["lat"]
                df_tmp[14] = global_position["lon"]

            if depth:
                df_tmp[15] = depth
        except (KeyError, TypeError) as exc:
            self.get_logger().warning(f"Unexpected Water Linked payload ({exc}); "
                                      "fields left at zero.", throttle_duration_sec=10.0)

        msg = Float32MultiArray()
        msg.data = [float(v) for v in df_tmp]
        self.uw_data_pub.publish(msg)


def main():
    rclpy.init()
    node = uw_gps_logger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as exc:                          # noqa: BLE001 - teardown race
        if rclpy.ok():
            raise
        print(f"uwgps_log: stopped during shutdown ({type(exc).__name__})", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
