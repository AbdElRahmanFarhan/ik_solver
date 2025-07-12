from __future__ import annotations

from urdf_parser_py import urdf
from typing import List, Tuple
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from utils import get_tf_from_dh, solve_for_delta_linear
import numpy as np


class IKSolver:
    def __init__(self, robot_urdf: Path):
        robot = urdf.Robot.from_xml_file(robot_urdf)
        self.dh_parameters = {
            "alpha": [0, -np.pi/2, -np.pi/2., 0., -np.pi/2, np.pi/2, -np.pi/2],
            "a": [0., 0., robot.joints[4].origin.xyz[0], robot.joints[5].origin.xyz[2], robot.joints[6].origin.xyz[2], 0., 0.],
            "d": [0., robot.joints[3].origin.xyz[2], 0., 0., robot.joints[7].origin.xyz[0], 0., 0.],
            "q": [-np.pi/2, -np.pi/2, -np.pi/2, 0., 0., 0., 0.]
        }
        self.T_7_EE = np.eye(4, 4)  # wrist as per DH to Flange
        self.T_7_EE[:3, :3] = R.from_euler("XYZ", [0., 0., np.pi]).as_matrix()
        self.T_7_EE[2, 3] = robot.joints[8].origin.xyz[0]

        self.T_B_G = np.eye(4, 4)  # base as per DH to Flange
        self.T_B_G[:3, :3] = R.from_euler(
            "XYZ", [np.pi, np.pi/2, 0.]).as_matrix()
        self.T_B_G[0, 3] = robot.joints[1].origin.xyz[0]
        self.T_B_G[1, 3] = robot.joints[1].origin.xyz[1]
        self.T_B_G[2, 3] = robot.joints[1].origin.xyz[2]
        self.theta1_limits = (
            robot.joints[0].limit.lower, robot.joints[0].limit.upper)
        self.robot_reach_radius = (1.0, 5.0)

    def forward_kinematics(self, thetas):
        T = np.eye(4, 4)
        for i, theta in enumerate(thetas):
            alpha = self.dh_parameters["alpha"][i]
            a = self.dh_parameters["a"][i]
            if i == 0:
                d = theta + self.dh_parameters["d"][i]
                q = self.dh_parameters["q"][i]
            else:
                d = self.dh_parameters["d"][i]
                q = theta + self.dh_parameters["q"][i]
            T = T @ get_tf_from_dh(alpha, a, d, q)
        return T

    def get_pose(self, thetas):
        T_G_7 = self.forward_kinematics(thetas)
        T_B_EE = self.T_B_G @ T_G_7 @ self.T_7_EE
        position = T_B_EE[:3, 3]
        quat = R.from_matrix(T_B_EE[:3, :3]).as_quat()
        pose = [*position, *quat]
        return pose

    def inverse_kinematics(self, pose: List[float], init_thetas=[0, 0, 0, 0, 0, 0, 0]) -> Tuple[float]:

        T_B_EE = np.eye(4)
        T_B_EE[:3, 3] = pose[:3]
        T_B_EE[:3, :3] = R.from_quat(pose[3:]).as_matrix()

        T_G_EE = np.eye(4)
        T_G_EE = np.linalg.inv(self.T_B_G) @ T_B_EE

        x = T_G_EE[0, 3]
        y = T_G_EE[1, 3]

        theta_init = init_thetas[0]

        delta_theta1 = solve_for_delta_linear(
            x, y, self.theta1_limits, self.robot_reach_radius, theta_init)

        theta1 = delta_theta1 + T_G_EE[2, 3]

        T_G_0 = np.eye(4)
        T_G_0[2, 3] = theta1
        T_0_7 = np.linalg.inv(
            T_G_0) @ np.linalg.inv(self.T_B_G) @ T_B_EE @ np.linalg.inv(self.T_7_EE)

        X = T_0_7[0, 3]
        Y = T_0_7[1, 3]
        Z = T_0_7[2, 3]

        theta2 = np.arctan2(-Y, Z)

        side_a = np.sqrt(
            self.dh_parameters["a"][4]**2 + self.dh_parameters["d"][4]**2)
        proj_z = X - self.dh_parameters["d"][1]
        proj_xy = np.sqrt(Z**2 + T_0_7[1, 3]**2) - self.dh_parameters["a"][2]
        side_b = np.sqrt(proj_z**2 + proj_xy**2)
        side_c = self.dh_parameters["a"][3]
        angle_a = np.arccos(
            (side_b * side_b + side_c * side_c - side_a * side_a)/(2 * side_b * side_c))
        angle_b = np.arccos(
            (side_a * side_a + side_c * side_c - side_b * side_b)/(2 * side_a * side_c))

        theta3 = np.pi/2. - angle_a - np.arctan2(proj_z, proj_xy)
        theta4 = np.pi - angle_b - \
            np.arctan2(self.dh_parameters["d"][4], self.dh_parameters["a"][4])

        T_G_4 = self.forward_kinematics([theta1, theta2, theta3, theta4])
        R_G_4 = T_G_4[:3, :3]

        R_Corr = R.from_euler("XYZ", [-np.pi/2, 0, 0]).as_matrix()
        R_4_7 = np.linalg.inv(R_G_4) @ np.linalg.inv(self.T_B_G[:3, :3]) @ T_B_EE[:3, :3] @ np.linalg.inv(
            self.T_7_EE[:3, :3]) @ np.linalg.inv(R_Corr)
        theta5, theta6, theta7 = R.from_matrix(R_4_7).as_euler("YZY")

        thetas = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]


        return thetas


def main():
    urdf_file = "/home/workspace/robot-urdfs/combined.urdf"

    ik_solver = IKSolver(urdf_file)

    poses_sampled_from_rviz = [
        [5.386, 0.430, 2.811, 0, 0.707, 0., 0.707],
        [2.956, 0.430, 4.297, 0.000, -0.125, 0.000, 0.992],
        [6.283, -2.692, 2.348, 0.070, 0.230, -0.340, 0.909],
        [4.058, -0.058, 3.423, 0.661, 0.553, -0.442, -0.248],
        [-0.885, -1.184, 3.424, 0.591, -0.578, 0.226, 0.515],
        [3.413, 3.156, 1.646, 0.424, -0.599, 0.103, 0.672]
    ]

    trials = []
    for pose in poses_sampled_from_rviz:

        thetas = ik_solver.inverse_kinematics(pose)
        pose_solution = ik_solver.get_pose(thetas)

        pose = np.array(pose).astype(float)
        pose_solution = np.array(pose_solution).astype(float)

        trials.append(np.allclose(pose, pose_solution, atol=1e-3))
    print(f"{sum(trials)} out of {len(trials)} succeeded")




if __name__ == '__main__':
    main()
