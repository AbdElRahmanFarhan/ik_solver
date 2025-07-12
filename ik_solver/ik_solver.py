from __future__ import annotations

from urdf_parser_py import urdf
from typing import List, Tuple
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from utils import get_tf_fixed_joint, get_tf_from_dh, solve_for_delta_linear
import numpy as np

class IKSolver:
    def __init__(self, robot_urdf: Path):
        self._robot = urdf.Robot.from_xml_file(robot_urdf)
        self.active_joints = [
            joint for joint in self._robot.joints if joint != "fixed"]
        self.dof = len(self.active_joints)
        self.dh_parameters = {
            "alpha": [0, -np.pi/2, -np.pi/2., 0., -np.pi/2, np.pi/2, -np.pi/2],
            "a": [0., 0., 0.320, 1.280, 0.200, 0., 0.],
            "d": [0., 0.78, 0., 0., 1.5925, 0., 0.],
            "q": [-np.pi/2, -np.pi/2, -np.pi/2, 0., 0., 0., 0.]
        }
        self.T_7_EE = np.eye(4, 4) # wrist as per DH to Flange
        self.T_7_EE[:3, :3] = R.from_euler("XYZ", [0., 0., np.pi]).as_matrix()
        self.T_7_EE[2, 3] = 0.2

        self.T_B_G = np.eye(4, 4) # base as per DH to Flange
        self.T_B_G[:3, :3] = R.from_euler("XYZ", [np.pi, np.pi/2, 0.]).as_matrix()
        self.T_B_G[0, 3] = 0.6
        self.T_B_G[1, 3] = 0.43
        self.T_B_G[2, 3] = 0.55

        
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
            T =  T @ get_tf_from_dh(alpha, a, d, q)
        return T
    
    def get_pose(self, thetas):
        T_G_7 = self.forward_kinematics(thetas)
        T_B_EE = self.T_B_G @ T_G_7 @ self.T_7_EE
        position = T_B_EE[:3, 3]
        quat = R.from_matrix(T_B_EE[:3, :3]).as_quat()
        pose = [*position, *quat]
        return pose

    
    def inverse_kinematics(self, pose: List[float], init_thetas) -> Tuple[float]:

        T_B_EE = np.eye(4)
        T_B_EE [:3, 3] = pose[:3]
        T_B_EE [:3, :3] = R.from_quat(pose[3:]).as_matrix()

        T_G_EE = np.eye(4)
        T_G_EE = np.linalg.inv(self.T_B_G) @ T_B_EE
 
        x = T_G_EE[0, 3]
        y = T_G_EE[1, 3]
        limits = (0.0, 5.0)
        robot_reach = (1.0, 5.0)
        theta_init = init_thetas[0]

        delta_theta1 = solve_for_delta_linear(x, y, limits, robot_reach, theta_init)

        theta1 = delta_theta1 + T_G_EE[2, 3]


        # after displacement
        T_G_0 = np.eye(4)
        T_G_0[2, 3] = theta1
        T_0_7 = np.linalg.inv(T_G_0) @ np.linalg.inv(self.T_B_G) @ T_B_EE @ np.linalg.inv(self.T_7_EE)

        X = T_0_7[0, 3]
        Y = T_0_7[1, 3]
        Z = T_0_7[2, 3]

        theta2 = np.arctan2(-Y, Z)
            
        side_a = np.sqrt(self.dh_parameters["a"][4]**2 + self.dh_parameters["d"][4]**2)
        proj_z = X - self.dh_parameters["d"][1]
        proj_xy = np.sqrt(Z**2 + T_0_7[1, 3]**2) - self.dh_parameters["a"][2]
        side_b = np.sqrt(proj_z**2 + proj_xy**2)
        side_c = self.dh_parameters["a"][3]
        angle_a = np.arccos((side_b * side_b + side_c * side_c - side_a * side_a)/(2 * side_b * side_c))
        angle_b = np.arccos((side_a * side_a + side_c * side_c - side_b * side_b)/(2 * side_a * side_c))

        theta3 = np.pi/2. - angle_a - np.arctan2(proj_z, proj_xy)
        theta4 = np.pi - angle_b - np.arctan2(self.dh_parameters["d"][4], self.dh_parameters["a"][4])

        
        T_G_4 = self.forward_kinematics([theta1, theta2, theta3, theta4])
        R_G_4 = T_G_4[:3, :3]
        
        R_Corr = R.from_euler("XYZ", [-np.pi/2, 0, 0]).as_matrix()
        R_4_7 = np.linalg.inv(R_G_4) @ np.linalg.inv(self.T_B_G[:3, :3]) @ T_B_EE[:3, :3] @ np.linalg.inv(self.T_7_EE[:3, :3]) @ np.linalg.inv(R_Corr)
        theta5, theta6, theta7 = R.from_matrix(R_4_7).as_euler("YZY")

        thetas = [theta1, theta2, theta3, theta4, theta5, theta6, theta7]

        return thetas





def main():
    urdf_file = "/home/workspace/robot-urdfs/combined.urdf"

    ik_solver = IKSolver(urdf_file)

    # pose = [5.386, 0.430, 2.811, 0, 0.707, 0., 0.707]
    pose = [2.956, 0.430, 4.297, 0.000, -0.125, 0.000, 0.992]
    init_thetas = [0, 0, 0, 0, 0, 0, 0]

    thetas = ik_solver.inverse_kinematics(pose, init_thetas)

    print(thetas)

    pose_solution = ik_solver.get_pose(thetas)

    print(pose_solution)
    pose = np.array(pose).astype(float)
    pose_solution_np = np.array(pose_solution).astype(float)

    print(np.allclose(pose, pose_solution_np, atol=1e-3))


if __name__ == '__main__':
    main()