from __future__ import annotations
from dataclasses import dataclass

from urdf_parser_py import urdf
from typing import List
from dataclasses import dataclass
from pathlib import Path
from sympy import *
import numpy as np
from scipy.spatial.transform import Rotation as R


def get_tf_from_dh(alpha: Symbol, a: Symbol, d: Symbol, q: Symbol) -> Matrix:
    tf = Matrix([[             cos(q),            -sin(q),            0,         a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                   0,                   0,            0,        1]])
    return tf

def get_tf_fixed_joint(joint: urdf.Joint) -> Matrix:
    rpy = joint.origin.rpy
    print(rpy)
    rot_matrix = R.from_euler("xyz", rpy).as_matrix()
    TF = Matrix.zeros(4, 4)
    TF[:3, :3] = rot_matrix
    TF[:3, 3] = joint.origin.xyz
    TF[-1, -1] = 1
    return TF