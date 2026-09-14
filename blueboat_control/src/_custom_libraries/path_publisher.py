#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from blueboat_interfaces.srv import RequestPath

"""
Asks path_generation for the whole path and republishes it on /set_path for
RViz and the GCS. Outside the control loop: master_control never reads this.

The request is REPEATED, not made once at construction. The trajectory served
by path_generation is not fixed for the run -- a from_yaml mission is
file-watched, so the deployed file may appear only after launch (that is what
makes a GPS-anchored mission possible: the station writes it once the run's
odom<->GPS fit is established) and may be edited mid-run. A one-shot request in
__init__ froze whatever existed at boot, which for every GPS-anchored mission
was the station-keeping fallback: a single dot at the origin for the whole run.

Nothing blocks in __init__ either -- neither on service discovery nor on a
response -- so the node reaches spin() even if path_generation is slow, starts
later, or never starts at all. The last good path keeps publishing while a new
request is in flight, so the display never blanks.
"""
class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Declare parameters
        self.declare_parameter('total_time', 1000.0)
        self.declare_parameter('dt', 0.1)
        # How often the whole path is re-requested. The service is cheap (every
        # shape evaluates in constant time per pose), and this is what picks up
        # a mission deployed or edited after launch.
        self.declare_parameter('refresh_period', 5.0)

        self.total_time = self.get_parameter('total_time').value
        self.dt = self.get_parameter('dt').value
        self.refresh_period = self.get_parameter('refresh_period').value

        self.publisher = self.create_publisher(Path, 'set_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.saved_path = Path()
        self.client = self.create_client(RequestPath, 'path_request')

        self.time_list = np.linspace(0, self.total_time,
                                     int(self.total_time / self.dt) + 1, dtype=float)
        self.future = None
        self.request_sent = None        # ROS time [s] the in-flight request went out
        self.reported_poses = None      # last pose count logged, to keep the log quiet
        self.reported_waiting = False

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _collect(self):
        """Take a completed response, if there is one."""
        if self.future is None:
            return
        if self.future.done():
            try:
                result = self.future.result()
                if result is not None:
                    self.saved_path = result.path
                    n = len(self.saved_path.poses)
                    if n != self.reported_poses:
                        self.get_logger().info(f'Received path with {n} poses.')
                        self.reported_poses = n
                else:
                    self.get_logger().error('Failed to call service.')
            except Exception as exc:
                self.get_logger().error(f'Path request raised: {exc}')
            self.future = None
        elif self._now() - self.request_sent > 3.0 * self.refresh_period:
            # A response that never arrives must not wedge the node.
            self.get_logger().warning('Path request timed out; retrying.')
            self.future = None

    def _request(self):
        """Issue the next whole-path request, if it is due and can be served."""
        if self.future is not None:
            return
        now = self._now()
        if self.request_sent is not None and now - self.request_sent < self.refresh_period:
            return
        if not self.client.service_is_ready():
            if not self.reported_waiting:
                self.get_logger().info('Waiting for service...')
                self.reported_waiting = True
            return
        self.reported_waiting = False
        request = RequestPath.Request()
        request.path_request.data = self.time_list
        self.future = self.client.call_async(request)
        self.request_sent = now

    def publish_path(self):
        self._collect()
        self._request()
        self.publisher.publish(self.saved_path)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
