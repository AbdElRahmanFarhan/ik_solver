from __future__ import annotations
from urdf_parser_py import urdf
import numpy as np
from scipy.spatial.transform import Rotation as R


def get_tf_from_dh(alpha, a, d, q):
    tf = np.array([[             np.cos(q),            -np.sin(q),            0,         a],
                [ np.sin(q)*np.cos(alpha), np.cos(q)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                [ np.sin(q)*np.sin(alpha), np.cos(q)*np.sin(alpha),  np.cos(alpha),  np.cos(alpha)*d],
                [                   0,                   0,            0,        1]])
    return tf

def get_tf_fixed_joint(joint: urdf.Joint):
    rpy = joint.origin.rpy
    rot_matrix = R.from_euler("xyz", rpy).as_matrix()
    TF = np.zeros(4, 4)
    TF[:3, :3] = rot_matrix
    TF[:3, 3] = joint.origin.xyz
    TF[-1, -1] = 1
    return TF