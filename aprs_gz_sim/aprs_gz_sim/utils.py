#!/usr/bin/env python3

import math

import math
from typing import Tuple
import PyKDL
from geometry_msgs.msg import (
    Pose,
    Quaternion
)


def convert_pi_string_to_float(s: str) -> float:
    """Takes a string that contains pi and evaluates the expression. Returns a float
    Returns 0.0 if the expression cannot be evaluated"""
    s=str(s)
    value = 0.0
    negative = False

    if s.isdigit():
        return float(s)

    if s.find('pi') == -1:
        # Return 0 if string does not contain pi
        return value

    if not s.find('-') == -1:
        negative = True
        s = s.replace('-', '')

    split = s.split('pi')
    if not len(split) == 2:
        # Can't evaluate strings with multiple pi's, return 0
        return value

    before, after = split
    if before and after:
        before = before.replace('*', '')
        if before.isdigit():
            value = float(before) * math.pi
        after = after.replace('/', '')
        if after.isdigit():
            value /= float(after)
    elif before:
        before = before.replace('*', '')
        if before.isdigit():
            value = float(before) * math.pi
    elif after:
        after = after.replace('/', '')
        if after.isdigit():
            value = math.pi / float(after)
    else:
        value = math.pi

    if negative:
        return -value
    else:
        return value

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def pose_info(xyz: list, rpy: list) -> Pose:
    xyz_floats = []
    rpy_floats = []
    for s in xyz:
        try:
            xyz_floats.append(float(s))
        except ValueError:
            xyz_floats.append(convert_pi_string_to_float(s))
    for s in rpy:
        try:
            rpy_floats.append(float(s))
        except ValueError:
            rpy_floats.append(convert_pi_string_to_float(s))

    pose = Pose()
    pose.position.x = xyz_floats[0]
    pose.position.y = xyz_floats[1]
    pose.position.z = xyz_floats[2]
    q = quaternion_from_euler(*rpy_floats)
    pose.orientation.w = q[0]
    pose.orientation.x = q[1]
    pose.orientation.y = q[2]
    pose.orientation.z = q[3]

    return pose

def multiply_pose(p1: Pose, p2: Pose) -> Pose:
    '''
    Use KDL to multiply two poses together.
    Args:
        p1 (Pose): Pose of the first frame
        p2 (Pose): Pose of the second frame
    Returns:
        Pose: Pose of the resulting frame
    '''

    o1 = p1.orientation
    frame1 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o1.x, o1.y, o1.z, o1.w),
        PyKDL.Vector(p1.position.x, p1.position.y, p1.position.z))

    o2 = p2.orientation
    frame2 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o2.x, o2.y, o2.z, o2.w),
        PyKDL.Vector(p2.position.x, p2.position.y, p2.position.z))

    frame3 = frame1 * frame2

    # return the resulting pose from frame3
    pose = Pose()
    pose.position.x = frame3.p.x()
    pose.position.y = frame3.p.y()
    pose.position.z = frame3.p.z()

    q = frame3.M.GetQuaternion()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def rpy_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
    ''' 
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        q (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: roll, pitch, yaw
    '''
    
    R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
    return R.GetRPY()

def build_pose(x,y,z,q : Quaternion)->Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation = q
    return p


def rad_to_deg_str(radians: float) -> str:
    '''
    Converts radians to degrees in the domain [-PI, PI]
    Args:
        radians (float): value in radians
    Returns:
        str: String representing the value in degrees
    '''
    
    degrees = math.degrees(radians)
    if degrees > 180:
        degrees = degrees - 360
    elif degrees < -180:
        degrees = degrees + 360

    if -1 < degrees < 1:
        degrees = 0 
    
    return f'{degrees:.0f}' + chr(176)

def rad_to_deg(radians: float) -> float:
    '''
    Converts radians to degrees
    Args:
        radians (float): Value in radians
    Returns:
        float: Value in degrees
    '''
    
    return radians * math.pi/180