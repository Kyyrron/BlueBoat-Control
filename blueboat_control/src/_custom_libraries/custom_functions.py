#!/usr/bin/env python3

import rclpy
# from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
# from nav_msgs.msg import Odometry
# from std_msgs.msg import String
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
import os
from scipy.interpolate import PchipInterpolator


def generate_interpolator():
    pwm = np.array([1100,1110,1136,1162,1188,1214,1240,1266,1292,1318,1344,1370,1396,1422,1448,1474,1500,1526,1552,1578,1604,1630,1656,1682,1708,1734,1760,1786,1812,1838,1864,1890,1900])
    thr = 9.80665*np.array([-2.81,-2.78,-2.64,-2.42,-2.21,-2.04,-1.83,-1.57,-1.42,-1.2,-0.98,-0.82,-0.6,-0.41,-0.24,-0.09,0,0.21,0.5,0.82,1.17,1.58,1.93,2.37,2.76,3.23,3.57,3.99,4.36,4.84,5.22,5.45,5.63])
    return PchipInterpolator(thr, pwm)

#################### Gazebo interaction ####################
def pause_gz(pause):
    os.system(f'gz service -s /world/ocean/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req \'pause: {pause}\' --timeout 1000')

def set_pose_gz(pose):
    x,y,z,phi,theta,psi = pose.astype(float)
    qx, qy, qz, qw = R.from_euler('xyz',[phi, theta, psi]).as_quat()
    
    pause_gz(True)
    os.system(f'gz service -s /world/ocean/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req \'name: "blueboat" position: {{ x: {x}, y: {y}, z: {z} }} orientation: {{ x: {qx}, y: {qy}, z: {qz}, w: {qw} }}\' --timeout 1000')
    pause_gz(False)

def set_current_gz(x, y, z):
    os.system(f'gz topic -t "/ocean_current" -m gz.msgs.Vector3d -p "x: {x}, y: {y}, z: {z}"')

#################### ROS2 interaction ####################
def odometry(msg, quat = False):
    # Extract pose
    msg_pose = msg.pose.pose

    # Extract position
    x = msg_pose.position.x
    y = msg_pose.position.y
    z = msg_pose.position.z

    # Extract orientation (quaternion)
    qx = msg_pose.orientation.x
    qy = msg_pose.orientation.y
    qz = msg_pose.orientation.z
    qw = msg_pose.orientation.w

    if quat: # Use quaternion directly
        pose = [x,y,z,qx,qy,qz,qw]

    else:
        # Convert quaternion to roll, pitch, yaw
        rot = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = rot.as_euler('xyz', degrees=False)

        pose = [x,y,z,roll,pitch,yaw]

    # Extract twist
    twist = msg.twist.twist

    u = twist.linear.x
    v = twist.linear.y
    w = twist.linear.z

    p = twist.angular.x
    q = twist.angular.y
    r = twist.angular.z

    twist = [u,v,w,p,q,r]

    return pose, twist

def compute_target(path, dt):

    def get_yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    # Extract current target and next step target
    poses = path.poses[:2]
    present = poses[0].pose
    future = poses[1].pose

    # Compute position
    x = future.position.x
    y = future.position.y
    psi = get_yaw_from_quaternion(future.orientation)
    psi = (psi + np.pi) % (2 * np.pi) - np.pi

    # Compute speeds
    dx = x - present.position.x
    dy = y - present.position.y
    u = np.hypot(dx, dy) / dt

    psi_prev = get_yaw_from_quaternion(present.orientation)

    psi_mid = (psi + psi_prev) / 2.0
    dx_b =  np.cos(psi_mid) * dx + np.sin(psi_mid) * dy
    dy_b = -np.sin(psi_mid) * dx + np.cos(psi_mid) * dy
    v = dy_b / dt 

    dpsi = (psi - psi_prev + np.pi) % (2 * np.pi) - np.pi
    r = dpsi / dt
    
    return [x, y, psi, u, v, r]

def planeFromQuaternion(inState):
    # Extract position
    x,y,z = inState[0:3]

    #Convert and extract euler angles
    phi,theta,psi = R.from_quat(inState[3:7]).as_euler('xyz')

    # Extract robot frame's speeds
    u,v,r = inState[7:]

    return np.array([x,y,psi,u,v,r]).reshape(-1,1)

def quaternion_multiply(q0, q1): # From https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion

def quaternion_error(q2, q1): # Returns "q2-q1"
    q1[3] *= -1 # Negate for inverse

    return quaternion_multiply(q2, q1)

def make_pose(pose_list, quat = False): #TODO: Make it usable in 3D
    pose = Pose()

    if quat:
        # Position
        pose.position.x = float(pose_list[0])
        pose.position.y = float(pose_list[1])
        pose.position.z = float(pose_list[2])

        pose.orientation.x = float(pose_list[3])
        pose.orientation.y = float(pose_list[4])
        pose.orientation.z = float(pose_list[5])
        pose.orientation.w = float(pose_list[6])

    else:
        x = pose_list[0]
        y = pose_list[1]
        theta = pose_list[2]

        # Position
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0

        # Orientation from yaw (theta)
        qz = np.sin(theta / 2.0)
        qw = np.cos(theta / 2.0)

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = qz
        pose.orientation.w = qw

    return pose

def create_pose_marker(inPose, inPub):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.5  # shaft length
    marker.scale.y = 0.05  # shaft diameter
    marker.scale.z = 0.05  # head diameter
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.pose = inPose

    marker.id = 0
    marker.lifetime.sec = 0  # persistent

    inPub.publish(marker)


#################### Trajectory generation ####################
def seabed_scanning(t):
    """
    Calculate non-differentiated reference positions at a single time t.
    
    Parameters
    ----------
    t : float
        Time at which to evaluate the trajectory.
    
    Returns
    -------
    xr, yr, zr, phir, thetar, psir : float
        Reference positions and orientations at time t.
    """
    velmin = 0.5
    r = velmin / (np.pi / 4)
    R = r
    rr = 1.0
    pas = 0.25

    # --- Positions ---
    if t <= 4:
        xr = velmin * t
        yr = 0.0
    elif t <= 6:
        xr = velmin*4 + r*np.sin((t-4)*np.pi/4)
        yr = r - r*np.cos((t-4)*np.pi/4)
    elif t <= 16:
        xr = r + velmin*4
        yr = r + velmin*(t-6)
    elif t <= 20:
        xr = velmin*4 + 2*r - r*np.cos((t-16)*np.pi/4)
        yr = r + velmin*10 + r*np.sin((t-16)*np.pi/4)
    elif t <= 30:
        xr = 3*r + velmin*4
        yr = r + velmin*10 - velmin*(t-20)
    elif t <= 40:
        xr = 3*r + velmin*4 + (velmin/np.sqrt(3))*(t-30)
        yr = r + (velmin/np.sqrt(3))*(t-30)
    elif t <= 40+12*np.pi:
        xr = 3*r + velmin*4 + 10*velmin/np.sqrt(3) + R*(1 + np.cos(np.pi + rr*velmin*(t-40)))
        yr = r + 10*velmin/np.sqrt(3) + R*np.sin(np.pi + rr*velmin*(t-40))
    else:
        # Default to last known point
        xr = 3*r + velmin*4 + 10*velmin/np.sqrt(3) - R
        yr = r + 10*velmin/np.sqrt(3)

    # --- Z position ---
    if t <= 30:
        zr = 1.0
    elif t <= 40:
        zr = 1.0 + (velmin/np.sqrt(3))*(t-30)
    elif t <= 40+4*np.pi:
        zr = 1.0 + 10*velmin/np.sqrt(3)
    elif t <= 40+12*np.pi:
        zr = 1.0 + 10*velmin/np.sqrt(3) - pas*velmin*(t-40-4*np.pi)
    else:
        zr = 1.0 + 10*velmin/np.sqrt(3) - pas*velmin*8*np.pi  # last known point

    # --- Rotations ---
    phir = 0.0  # Roll is zero in all phases

    if t <= 30:
        thetar = 0.0
    elif t <= 40:
        thetar = -np.arcsin(1/np.sqrt(3))
    elif t <= 40+4*np.pi:
        thetar = -np.pi/6
    elif t <= 40+12*np.pi:
        thetar = -np.pi/6 + (np.pi/3)*((t-40-4*np.pi)/(8*np.pi))
    else:
        thetar = np.pi/6  # last known

    if t <= 4:
        psir = 0.0
    elif t <= 6:
        psir = (t-4)*np.pi/4
    elif t <= 16:
        psir = np.pi/2
    elif t <= 20:
        psir = np.pi/2 - (t-16)*np.pi/4
    elif t <= 30:
        psir = -np.pi/2
    elif t <= 40:
        psir = np.pi/4
    elif t <= 40+12*np.pi:
        psir = rr*velmin*(t-40)
    else:
        psir = rr*velmin*12*np.pi 

    return xr, yr, zr, phir, thetar, psir

def quaternion_to_yaw(q: Quaternion):
    # yaw (Z axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quaternion(yaw: float):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def transform_body_to_world(x_r, y_r, yaw, x_b, y_b):
    c = math.cos(yaw)
    s = math.sin(yaw)

    x_w = x_r + (c * x_b - s * y_b)
    y_w = y_r + (s * x_b + c * y_b)

    return x_w, y_w

def enu_to_gps(lat0_deg, lon0_deg, east, north):
    EARTH_RADIUS = 6378137.0  # meters

    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)

    dlat = north / EARTH_RADIUS
    dlon = east / (EARTH_RADIUS * math.cos(lat0))

    lat = lat0 + dlat
    lon = lon0 + dlon

    return math.degrees(lat), math.degrees(lon)

def local_to_enu(x, y, yaw0):
    # rotate local frame into ENU
    theta = yaw0 - math.pi / 2.0

    c = math.cos(theta)
    s = math.sin(theta)

    east  = c * x - s * y
    north = s * x + c * y

    return east, north