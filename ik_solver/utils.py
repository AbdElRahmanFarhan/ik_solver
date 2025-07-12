from __future__ import annotations
from urdf_parser_py import urdf
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize


def get_tf_from_dh(alpha, a, d, q):
    tf = np.array([[np.cos(q),            -np.sin(q),            0,         a],
                   [np.sin(q)*np.cos(alpha), np.cos(q) *
                    np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                   [np.sin(q)*np.sin(alpha), np.cos(q)*np.sin(alpha),
                    np.cos(alpha),  np.cos(alpha)*d],
                   [0,                   0,            0,        1]])
    return tf


def solve_for_delta_linear(x, y, z_limits, r_limits, z_init):
    def objective(vars): return (vars[0] - z_init)**2
    def constraint_eq(vars): return vars[0] - np.sqrt(vars[1]**2 - x**2 - y**2)
    def constraint_ineq_real(vars): return vars[1]**2 - x**2 - y**2

    bounds = [z_limits, r_limits]
    z_r_init = [z_init, np.mean(r_limits)]
    constraints = [
        {'type': 'eq',  'fun': constraint_eq},
        {'type': 'ineq', 'fun': constraint_ineq_real},  # realness
    ]

    result = minimize(objective, z_r_init, method='SLSQP',
                      bounds=bounds, constraints=constraints)

    z_sol, _ = result.x
    return z_sol
