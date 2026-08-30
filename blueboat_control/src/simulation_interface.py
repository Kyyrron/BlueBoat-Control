#!/usr/bin/env python3

# rclpy
from rclpy.node import Node
import rclpy

# Common python libraries
import time
# import numpy as np

# ROS2 msg libraries
from std_msgs.msg import Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

# Custom libraries
from blueboat_control import ROV
import custom_functions as cf

class Controller(Node):
    def __init__(self):

        super().__init__('pid_sim', namespace='blueboat')

        self.declare_parameter('controller_type', 'MPC') 
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        # Loss-of-reference watchdog, same name, semantics and default as
        # robot_interface: simulation and real water must not differ on a safety
        # property. master_control publishes /thruster_input at 20 Hz, so 0.5 s is
        # ten missed producer ticks (five ticks of this node's own 10 Hz loop).
        self.declare_parameter('thruster_input_timeout', 0.5)
        self.thruster_input_timeout = self.get_parameter('thruster_input_timeout').get_parameter_value().double_value

        self.rov = ROV(self, thrust_visual = True)

        self.odom_sim_subscriber = self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.thruster_input_sub = self.create_subscription(Float32MultiArray, "/thruster_input", self.thr_input_callback,10)

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_publisher = self.create_publisher(Bool, '/blueboat/controller_ready', latched)
        self.data_publisher = self.create_publisher(Float32MultiArray, "/monitoring_data", 10)

        self.timer = self.create_timer(0.1, self.move)

        self.thr_input = [0,0]

        # None until the first /thruster_input arrives, which is itself a
        # "no reference" condition.
        self.last_thr_rx = None
        self.thr_watchdog_tripped = False

        self.sent_ready = False

        # Initialize monitoring values
        # self.monitoring = []
        # self.monitoring.append(['x','y','psi','x_d','y_d','psi_d','u1','u2','t'])

        # ctrl = self.controller_type
        # date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        # # self.title = 'data/MPC_data/' + self.date + '-mpc_data'
        # self.title = f'data/{ctrl}_data/{date}-{ctrl}_data'

    def thr_input_callback(self, msg: Float32MultiArray):
        self.thr_input = msg.data
        self.last_thr_rx = time.time()

    def thruster_input_stale(self, now):
        """
        Loss-of-reference watchdog predicate - the mirror of
        robot_interface.thruster_input_stale. True when /thruster_input has gone
        quiet for longer than thruster_input_timeout, so the last received thrust
        is not re-applied to the Gazebo thrusters indefinitely.
        """
        if self.last_thr_rx is None:
            return True
        return (now - self.last_thr_rx) > self.thruster_input_timeout

    def odom_callback(self, msg: Odometry):
        pose, twist = cf.odometry(msg)

        self.rov.current_pose = pose
        self.rov.current_twist = twist

    def move(self):
        if not self.rov.ready():
            return

        if not self.sent_ready:
            msg = Bool()
            msg.data = True
            self.ready_publisher.publish(msg)
            self.get_logger().info(f'Ready publishing: {msg.data}')
            self.sent_ready = True

        if self.thruster_input_stale(time.time()):
            if not self.thr_watchdog_tripped:
                self.thr_watchdog_tripped = True
                self.get_logger().warn(
                    f"No /thruster_input for {self.thruster_input_timeout:.2f} s "
                    "- zeroing thrust.")
            self.thr_input = [0,0]
        elif self.thr_watchdog_tripped:
            self.thr_watchdog_tripped = False
            self.get_logger().info("/thruster_input resumed - releasing watchdog.")

        r,l = self.thr_input
        # Apply force to thrusters
        self.rov.move([r,l])

rclpy.init()
node = Controller()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()