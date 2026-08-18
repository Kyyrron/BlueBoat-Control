#!/usr/bin/env python3

# rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSDurabilityPolicy
import rclpy

# Common python libraries
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from datetime import datetime

# ROS2 msg libraries
from std_msgs.msg import String, Float32, Float32MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker

# Custom libraries
from urdf_parser_py import urdf
import uvr_mpc
from blueboat_control import ROV
from blueboat_interfaces.srv import RequestPath
import custom_functions as cf

class Controller(Node):
    def __init__(self):

        super().__init__('mpc_control', namespace='blueboat')

        self.rov = ROV(self, thrust_visual = True)

        self.odom_sim_subscriber = self.create_subscription(Odometry, '/blueboat/odom', self.odom_callback, 10)
        self.pose_arrow_publisher = self.create_publisher(Marker, "/pose_arrow", 10)
        self.data_publisher = self.create_publisher(Float32MultiArray, "/monitoring_data", 10)

        # Create a client for path request
        self.client = self.create_client(RequestPath, '/path_request')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")
        
        self.future = None # Used for client requests

        self.timer = self.create_timer(0.01, self.move)

        # MPC Parameters
        self.mpc_horizon = 10
        self.mpc_time = 2.0
        self.mpc_path = Path()
        linear_bound = 40.0
        angular_bound = 15.0
        self.input_bounds = {"lower": np.array([-linear_bound, -linear_bound, -linear_bound]),
                             "upper": np.array([linear_bound, linear_bound, linear_bound]),
                             "idx":   np.array([0, 1, 2])
                             }
        self.Q_weight = np.diag([50, # x
                                 50, # y 
                                 40, # psi
                                 1, # u
                                 1, # v
                                 1  # r
                                 ])
        
        self.R_weight = np.diag([0.015, # u1
                                 0.015, # u2
                                 0.015  # u3
                                 ])

        # Initialize MPC solver
        self.controller = None #Updated at the start of spin

        # Initialize monitoring values
        self.monitoring = []
        self.monitoring.append(['x','y','psi','x_d','y_d','psi_d','u1','u2','u3','t'])

        self.date = datetime.today().strftime('%Y_%m_%d-%H_%M_%S')
        self.title = 'data/MPC_data/' + self.date + '-mpc_data'

        self.t_record = self.get_time()

    def get_time(self):
        s,ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns*1e-9

    def odom_callback(self, msg: Odometry):
        pose, twist = cf.odometry(msg)

        self.rov.current_pose = pose
        self.rov.current_twist = twist

    def move(self):
        if not self.rov.ready():
            return

        if self.controller is None:
            self.controller = uvr_mpc.MPCController(robot_mass = self.rov.mass,
                                            iz = self.rov.inertia[-1], 
                                            a_u = self.rov.added_masses[0],
                                            a_v = self.rov.added_masses[1],
                                            a_r = self.rov.added_masses[5],
                                            d_u = self.rov.viscous_drag[0],
                                            d_v = self.rov.viscous_drag[1],
                                            d_r = self.rov.viscous_drag[5],
                                            horizon = self.mpc_horizon, 
                                            time = self.mpc_time, 
                                            Q_weight = self.Q_weight,
                                            R_weight = self.R_weight,
                                            input_bounds = self.input_bounds
                                            )

        t = self.get_time()

        # Check if previous future is still pending
        if self.future is not None:
            if self.future.done():
                try:
                    result = self.future.result()
                    if result is not None:
                        self.mpc_path = result.path
                        # self.get_logger().info(f"Received path with {len(self.mpc_path.poses)} poses.")
                    else:
                        self.get_logger().error("Service returned None.")
                except Exception as e:
                    self.get_logger().error(f"Service call raised exception: {e}")
                finally:
                    self.future = None
                return

        # Send new request
        request = RequestPath.Request()
        request.path_request.data = np.linspace(t, t + self.mpc_time, int(self.mpc_horizon), dtype=float)

        self.future = self.client.call_async(request)

        # MPC control
        u = np.zeros(3)

        if self.rov.current_pose is not None and self.rov.current_twist is not None:
            x_current = np.array([self.rov.current_pose[0], # x
                                  self.rov.current_pose[1], # y
                                  self.rov.current_pose[5], # yaw
                                  self.rov.current_twist[0], # u
                                  self.rov.current_twist[1], # v
                                  self.rov.current_twist[5]]) # r

        x_current = np.array(x_current).reshape(-1)

        if self.mpc_path.poses: # Make sure the path is not empty

            self.mpc_path.poses

            desired_pose = self.mpc_path.poses[0].pose
            cf.create_pose_marker(desired_pose, self.pose_arrow_publisher) # Display the current desired pose

            u = self.controller.solve(path=self.mpc_path, x_current=x_current) 

        # Apply force to thrusters
        self.rov.move([u[0],u[1],u[2]])

        # Update and save monitoring metrics to be graphed later
        if self.mpc_path.poses:
            x_m = x_current[0]
            y_m = x_current[1]
            psi_m = x_current[2]

            x_d_m = desired_pose.position.x
            y_d_m = desired_pose.position.y

            q = desired_pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            psi_d_m = math.atan2(siny_cosp, cosy_cosp)

            data_array = [x_m, y_m, psi_m, x_d_m, y_d_m , psi_d_m, u[0],u[1],u[2], t]

            self.monitoring.append(data_array)

            publisher_msg = Float32MultiArray()
            publisher_msg.data = data_array
            self.data_publisher.publish(publisher_msg)
            # self.get_logger().info(f'Publishing: {msg.data}')

            if (t - self.t_record) > 1: # Update the saved file every second as doing so every step may corrupt the file if the callback is too frequent
                self.t_record = t
                np.save(self.title, self.monitoring)
        

rclpy.init()
node = Controller()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()