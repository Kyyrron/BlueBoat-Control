#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from blueboat_interfaces.srv import RequestPath

"""
Sends a client request to path_generation and saves the full path msg, then publishes it to rviz
"""
class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Declare parameters
        self.declare_parameter('total_time', 1000.0)
        self.declare_parameter('dt', 0.1)

        self.total_time = self.get_parameter('total_time').value
        self.dt = self.get_parameter('dt').value

        self.publisher = self.create_publisher(Path, 'set_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.saved_path = Path()

        # Create a client and request the full path to be saved and published
        self.client = self.create_client(RequestPath, 'path_request')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")


        time_list = np.linspace(0, self.total_time, int(self.total_time/self.dt)+1, dtype=float)

        request = RequestPath.Request()
        request.path_request.data = time_list
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.saved_path = future.result().path
            self.get_logger().info(f'Received path with {len(self.saved_path.poses)} poses.')
        else:
            self.get_logger().error('Failed to call service.')

    def publish_path(self):
        self.publisher.publish(self.saved_path)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()