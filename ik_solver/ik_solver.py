from __future__ import annotations

from urdf_parser_py import urdf
from typing import List, Tuple
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from utils import get_tf_fixed_joint, get_tf_from_dh
import numpy as np

class IKSolver:
    def __init__(self, robot_urdf: Path):
        self._robot = urdf.Robot.from_xml_file(robot_urdf)
        self.active_joints = [
            joint for joint in self._robot.joints if joint != "fixed"]
        self.dof = len(self.active_joints)
        self.dh_parameters = {
            "alpha": [0, -np.pi/2, 0., -np.pi/2, np.pi/2, -np.pi/2, 0],
            "a": [0., 0.320, 1.280, 0.200, 0., 0., 0.],
            "d": [0.78, 0., 0., 1.5925, 0., 0., 0.20],
            "q": [0., -np.pi/2, 0., 0., 0., 0., np.pi]
        }
        self.T_W_F = np.eye(4, 4) # Wrist in the DH to Flange
        self.T_W_F[:3, :3] = R.from_euler("XYZ", [self.dh_parameters["alpha"][3], 0, self.dh_parameters["q"][6]]).as_matrix()
        self.T_W_F[2, 3] = self.dh_parameters["d"][6]

        
    # def _gantry_FK(self) -> Matrix:
    #     q0 = Symbol('q0')
    #     prismatic_joint = self._robot.joints[0]
    #     T_B_L1 = get_tf_fixed_joint(prismatic_joint)
    #     displacement = Matrix([q0 * a for a in prismatic_joint.axis])
    #     T_B_L1[:3, 3] += displacement

    #     flange_joint = self._robot.joints[1]
    #     T_L1_F = get_tf_fixed_joint(flange_joint)
    #     T_B_F = T_B_L1 @ T_L1_F
    #     return T_B_F
    

    # def forward_kinematics(self, q) -> List[float]:
    #     q_symbols = symbols('q0:8')
    #     subs = {q_symbol: qi for q_symbol, qi in zip(q_symbols, q)}
    #     print(subs)
    #     T0_EE = self.robot_FK.subs(subs)
    #     T_B_F = self.gantry_FK.subs(subs)
    #     T_F_0 = get_tf_fixed_joint(self._robot.joints[2])
    #     TB_EE = T_B_F @ T_F_0 @ T0_EE
    #     position = TB_EE[:3, 3]
    #     quat = R.from_matrix(TB_EE[:3, :3]).as_quat()
    #     return position, quat

    # def forward_kinematics(self, q) -> List[float]:
    #     subs = {q_symbol: qi for q_symbol, qi in zip(q_symbols, q)}
    #     T0_EE = self.robot_FK.subs(subs)
    #     position = list(T0_EE[:3, 3])
    #     quat = R.from_matrix(T0_EE[:3, :3]).as_quat()
    #     pose = [*position, *quat]
    #     return pose    
    
    def forward_kinematics(self, qs):
        T = np.eye(4, 4)
        for i, q in enumerate(qs):
            alpha = self.dh_parameters["alpha"][i]
            a = self.dh_parameters["a"][i]
            d = self.dh_parameters["d"][i]
            q = q + self.dh_parameters["q"][i]
            T =  T @ get_tf_from_dh(alpha, a, d, q)
        return T
    
    def get_pose(self, qs):
        qs.append(0) # for the transormation to the flange
        T = self.forward_kinematics(qs)
        position = T[:3, 3]
        quat = R.from_matrix(T[:3, :3]).as_quat()
        pose = [*position, *quat]
        return pose

    
    def inverse_kinematics(self, pose: List[float]) -> Tuple[float]:

        position = pose[:3]
        quat = pose[3:]

        R_0_EE = R.from_quat(quat).as_matrix()
        
        wrist_center = position - self.dh_parameters["d"][6] * R_0_EE[:, 2]
        theta1 = np.arctan2(wrist_center[1], wrist_center[0])
            
        side_a = np.sqrt(self.dh_parameters["a"][3]**2 + self.dh_parameters["d"][3]**2)
        proj_z = wrist_center[2] - self.dh_parameters["d"][0]
        proj_xy = np.sqrt(wrist_center[0]**2 + wrist_center[1]**2) - self.dh_parameters["a"][1]
        side_b = np.sqrt(proj_z**2 + proj_xy**2)
        side_c = self.dh_parameters["a"][2]
        angle_a = np.arccos((side_b * side_b + side_c * side_c - side_a * side_a)/(2 * side_b * side_c))
        angle_b = np.arccos((side_a * side_a + side_c * side_c - side_b * side_b)/(2 * side_a * side_c))

        theta2 = np.pi/2. - angle_a - np.arctan2(proj_z, proj_xy)
        theta3 = np.pi - angle_b - np.arctan2(self.dh_parameters["d"][3], self.dh_parameters["a"][3])

        
        T_0_W = self.forward_kinematics([theta1, theta2, theta3])
        R_0_W = T_0_W[:3, :3]
         
        R_W_EE = np.linalg.inv(R_0_W) @ R_0_EE @ np.linalg.inv(self.T_flange[:3, :3])

        theta4, theta5, theta6 = R.from_matrix(R_W_EE).as_euler("YZY")

        thetas = [theta1, theta2, theta3, theta4, theta5, theta6]

        return thetas




def main():
    urdf_file = "/home/workspace/robot-urdfs/combined.urdf"

    ik_solver = IKSolver(urdf_file)

    pose = [0.627, 3.032, 1.894, 0.149, 0.761, 0.436, 0.456]


    qs = ik_solver.inverse_kinematics(pose)
    pose_solution = ik_solver.get_pose(qs)

    pose = np.array(pose).astype(float)
    pose_solution_np = np.array(pose_solution).astype(float)

    print(np.allclose(pose, pose_solution_np, atol=1e-3))


if __name__ == '__main__':
    main()