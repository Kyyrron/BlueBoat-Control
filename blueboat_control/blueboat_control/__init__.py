#!/usr/bin/env python3

from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from urdf_parser_py import urdf
import numpy as np
import casadi as ca
import sympy as sp


def convert(v):
    return np.array([v.x,v.y,v.z])


class ROV:

    def __init__(self, node: Node,
                 thrust_visual = False,
                 PID = False):

        self.node = node
        self.display_wrench = thrust_visual # Creating and storing the bool so it may be accessed and changed during simulation if the need arises
        self.PID = PID
        self.robot_sub = node.create_subscription(String, 'robot_description',
                                                self.read_model,
                                                QoSProfile(depth=1,
                                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # Pose
        self.current_pose = None
        self.current_twist = None

        # state feedback
        self.p = None
        self.R = None
        self.v = None
        self.w = None

        self.odom_sub = node.create_subscription(Odometry, 'odom', self.odom_cb, 1)
        self.js_sub = node.create_subscription(JointState, 'joint_states', self.joint_cb, 1)

        # Initialize attributes

        self.thruster_pub = []
        self.wrench_pub = []
        self.joint_pub = []
        self.joints = []

        self.mass = None
        self.added_masses = None
        self.viscous_drag = None
        self.quadratic_drag = None

        self.rg = None
        self.rb = None

        self.inertia = None

    def parsed(self):
        return self.thruster_pub

    def ready(self):
        return self.rb is not None and self.current_pose is not None

    def odom_cb(self, odom: Odometry):

        self.p = convert(odom.pose.pose.position)
        q = odom.pose.pose.orientation
        self.R = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()

        self.v = convert(odom.twist.twist.linear)
        self.w = convert(odom.twist.twist.angular)

    def joint_cb(self, joints: JointState):

        for i,thruster in enumerate(self.joints):
            if thruster in joints.name:
                idx = joints.name.index(thruster)
                self.q[i] = joints.position[idx]

    def move(self, forces, angles=0):

        msg = Float64()
        if not self.PID:
            for i, val in enumerate(forces):
                msg.data = float(val)
                self.thruster_pub[i].publish(msg)
        # for i, val in enumerate(angles):
        #     msg.data = float(val)
        #     self.joint_pub[i].publish(msg)

        # Redundant again but future me might thank me
        if self.display_wrench:
            wrench_msg = WrenchStamped()
            for i, val in enumerate(forces):
                wrench_msg.header.frame_id = "blueboat/thruster"+str(i+1)
                # Create and publish WrenchStamped
                wrench_msg.wrench.force.x = float(val)
                wrench_msg.wrench.force.y = 0.0
                wrench_msg.wrench.force.z = 0.0
                wrench_msg.wrench.torque.x = 0.0
                wrench_msg.wrench.torque.y = 0.0
                wrench_msg.wrench.torque.z = 0.0
                self.wrench_pub[i].publish(wrench_msg)

            # Base link (used to easily see the robot's orientation)
            wrench_msg = WrenchStamped()
            wrench_msg.header.frame_id = "blueboat/base_link"
            wrench_msg.wrench.force.x = 10.0
            wrench_msg.wrench.force.y = 0.0
            wrench_msg.wrench.force.z = 0.0
            wrench_msg.wrench.torque.x = 0.0
            wrench_msg.wrench.torque.y = 0.0
            wrench_msg.wrench.torque.z = 0.0
            self.wrench_pub[-1].publish(wrench_msg)

            

    def read_model(self, msg):

        if len(self.joints):
            return

        robot: urdf.Robot = urdf.Robot.from_xml_string(msg.data)

        # identify thrusters
        thrusters = [j.name for j in robot.joints if j.type == 'continuous']
        thrusters.sort()
        for thr in thrusters:
            self.thruster_pub.append(self.node.create_publisher(Float64, 'cmd_'+thr, 1))
            # Wrench publishers, used to display thruster inputs,
            self.wrench_pub.append(self.node.create_publisher(WrenchStamped, 'blueboat_' + thr + '_wrench', 1))

        self.wrench_pub.append(self.node.create_publisher(WrenchStamped, 'blueboat_base', 1))

        # print('Thrusters:', thrusters)

        root = robot.get_root()
        base_link = robot.link_map[root]
        # print('mass:',l1.inertial.mass)
        # print('rg:', l1.inertial.origin)
        # print('inertia:', l1.inertial.inertia)

        # Read the robot's dynamic parameters
        Ma = [0]*6
        Dl = [0]*6
        Dq = [0]*6

        for gz in robot.gazebos:
            for plugin in gz.findall('plugin'):
                if 'Hydrodynamics' in plugin.get('name'):
                    for i,(axis,force) in enumerate(('xU','yV','zW', 'kP', 'mQ', 'nR')):
                        for tag in plugin.findall(axis+force):
                            Dl[i] = float(tag.text)
                        for tag in plugin.findall(f'{axis}{force}abs{force}'):
                            Dq[i] = float(tag.text)
                        for tag in plugin.findall(f'{axis}Dot{force}'):
                            Ma[i] = float(tag.text)

        # Update robot's model for hydrodynamic computation
        self.mass = base_link.inertial.mass
        self.added_masses = Ma
        self.viscous_drag = Dl
        self.quadratic_drag = Dq

        self.rg = base_link.inertial.origin.xyz
        self.rb = base_link.collision.origin.xyz

        read_inertia = base_link.inertial.inertia
        self.inertia = [
            read_inertia.ixx,
            read_inertia.ixy,
            read_inertia.ixz,
            read_inertia.iyy,
            read_inertia.iyz,
            read_inertia.izz
        ]

        # build thruster allocation matrix
        def sk(u):
            return sp.Matrix([[0,-u[2],u[1]],[u[2],0,-u[0]],[-u[1],u[0],0]])

        def Rot(theta,u):
            R = sp.cos(theta)*sp.eye(3) + sp.sin(theta)*sk(u) + (1-sp.cos(theta))*(u*u.transpose())
            return sp.Matrix(R)

        X = sp.Matrix([1,0,0]).reshape(3,1)
        Y = sp.Matrix([0,1,0]).reshape(3,1)
        Z = sp.Matrix([0,0,1]).reshape(3,1)

        def simp_matrix(M):
            '''
            simplify matrix for old versions of sympy
            '''
            for i in range(M.rows):
                for j in range(M.cols):
                    M[i,j] = sp.trigsimp(M[i,j])
            return M

        def Homogeneous(t, R):
            '''
            Homogeneous frame transformation matrix from translation t and rotation R
            '''
            return (R.row_join(t)).col_join(sp.Matrix([[0,0,0,1]]))

        def extract(parent, joint):

            def simp_val(xyz):
                for i in range(3):
                    for v in (-1,0,1):
                        if abs(xyz[i]-v) < 1e-5:
                            xyz[i] = v
                return xyz

            # assumes only last joint is non-fixed
            joints = robot.get_chain(parent, joint.child)[1::2]
            M = sp.eye(4)
            for name in joints:
                joint = robot.joint_map[name]
                if joint.origin is None:
                    continue
                if joint.origin.xyz is None:
                    xyz = sp.zeros(3,1)
                else:
                    xyz = simp_val(sp.Matrix(joint.origin.xyz))
                if joint.origin.rpy is None:
                    rpy = sp.zeros(3,1)
                else:
                    rpy = sp.Matrix(joint.origin.rpy)
                    for i in range(3):
                        for k in range(-12,13):
                            if abs(rpy[i] - k*np.pi/12.) < 1e-5:
                                rpy[i] = str(sp.simplify(k*sp.pi/12))
                                if rpy[i] == '0':
                                    rpy[i] = 0
                M = M * Homogeneous(xyz, Rot(rpy[2],Z)*Rot(rpy[1],Y)*Rot(rpy[0],X))

            # also output final axis
            if joint.axis is None:
                axis = [1,0,0]
            else:
                axis = np.array(joint.axis)
            axis = simp_val(sp.Matrix(axis/np.linalg.norm(axis)))

            return M, sp.Matrix(axis).reshape(3,1)

        # identify thruster joints and build TAM
        T = sp.zeros(6,0)
        q = {}

        for thr in thrusters:

            # root to steering
            chain = robot.get_chain(root, robot.joint_map[thr].child)[1::2]
            joints = [robot.joint_map[j] for j in chain if robot.joint_map[j].type != 'fixed']

            if len(joints) == 2:
                # steering part
                name = joints[0].name
                if name not in q:
                    q[name] = sp.symbols(name)

                Ms, axis = extract(root, joints[0])
                M = Ms * Homogeneous(0*X, Rot(q[name], axis))
                Ms, axis = extract(joints[0].child,joints[1])
                M *= Ms
            else:
                M, axis = extract(root, joints[0])

            # corresponding TAM column
            t,R = M[:3,3], M[:3,:3]
            T = T.row_join((R*axis).col_join(sk(t)*R*axis))

        self.joints = sorted(q.keys())
        for joint in self.joints:
            self.joint_pub.append(self.node.create_publisher(Float64, 'cmd_'+joint, 1))

        # print('Thrusters steering:', self.joints)

        # to Casadi symbols function
        # joint states
        self.qs = [ca.SX.sym(name) for name in self.joints]
        # thruster allocation matrix
        self.TAM = sp.lambdify([q[name] for name in self.joints], T)(*self.qs)

        # print(self.TAM)

        # numerical values from joint states
        self.q = [0 for _ in self.qs]

        TAM_num = sp.lambdify([q[name] for name in self.joints], T, modules='numpy')
        self.B = np.array(TAM_num(*self.q), dtype=np.float64)
