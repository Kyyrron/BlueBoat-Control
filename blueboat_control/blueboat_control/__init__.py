#!/usr/bin/env python3

from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import WrenchStamped
from urdf_parser_py import urdf


class ROV:
    """The simulated boat's thrusters, as the robot description names them:
    one `cmd_<thruster>` force topic each (alphabetical joint order, so
    forces[0] drives thruster1, the right one) and, with `thrust_visual`, a
    `blueboat_<thruster>_wrench` arrow each plus `blueboat_base`."""

    def __init__(self, node: Node, thrust_visual=False):
        self.node = node
        self.display_wrench = thrust_visual
        self.robot_sub = node.create_subscription(
            String, 'robot_description', self.read_model,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.current_pose = None        # set by simulation_interface
        self.current_twist = None
        self.thruster_pub = []
        self.wrench_pub = []

    def parsed(self):
        return self.thruster_pub

    def ready(self):
        return bool(self.thruster_pub) and self.current_pose is not None

    def move(self, forces):
        msg = Float64()
        for i, val in enumerate(forces):
            msg.data = float(val)
            self.thruster_pub[i].publish(msg)

        if self.display_wrench:
            for i, val in enumerate(forces):
                wrench_msg = WrenchStamped()
                wrench_msg.header.frame_id = "blueboat/thruster" + str(i + 1)
                wrench_msg.wrench.force.x = float(val)
                self.wrench_pub[i].publish(wrench_msg)
            # Base link (to see the robot's orientation easily)
            wrench_msg = WrenchStamped()
            wrench_msg.header.frame_id = "blueboat/base_link"
            wrench_msg.wrench.force.x = 10.0
            self.wrench_pub[-1].publish(wrench_msg)

    def read_model(self, msg):
        if self.thruster_pub:
            return
        robot = urdf.Robot.from_xml_string(msg.data)
        thrusters = sorted(j.name for j in robot.joints if j.type == 'continuous')
        for thr in thrusters:
            self.thruster_pub.append(self.node.create_publisher(Float64, 'cmd_' + thr, 1))
            self.wrench_pub.append(self.node.create_publisher(WrenchStamped, 'blueboat_' + thr + '_wrench', 1))
        self.wrench_pub.append(self.node.create_publisher(WrenchStamped, 'blueboat_base', 1))
