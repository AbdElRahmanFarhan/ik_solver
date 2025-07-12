from __future__ import annotations

from urdf_parser_py import urdf
from typing import List, Tuple
from pathlib import Path
from sympy import *
from scipy.spatial.transform import Rotation as R
from utils import get_tf_fixed_joint, get_tf_from_dh
import numpy as np

class IKSolver:
    def __init__(self, robot_urdf: Path):
        self._robot = urdf.Robot.from_xml_file(robot_urdf)
        self.active_joints = [
            joint for joint in self._robot.joints if joint != "fixed"]
        self.dof = len(self.active_joints)
        self.robot_FK = self._robot_FK()
        self.gantry_FK = self._gantry_FK()
        self.robot_FK_3 = self._robot_FK_3()
        
    def _gantry_FK(self) -> Matrix:
        q0 = Symbol('q0')
        prismatic_joint = self._robot.joints[0]
        T_B_L1 = get_tf_fixed_joint(prismatic_joint)
        displacement = Matrix([q0 * a for a in prismatic_joint.axis])
        T_B_L1[:3, 3] += displacement

        flange_joint = self._robot.joints[1]
        T_L1_F = get_tf_fixed_joint(flange_joint)
        T_B_F = T_B_L1 @ T_L1_F
        return T_B_F
    

    def _robot_FK(self):
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        dh_parameters = {alpha0:        0,  a0:         0,  d1:      0.780,
                    alpha1: -pi/2.,  a1:      0.320,  d2:         0, q2: q2 - pi/2.,
                    alpha2:        0,  a2:      1.280,  d3:         0,
                    alpha3: -pi/2.,  a3: 0.200,  d4:       1.5925,
                    alpha4:     pi/2.,  a4:         0,  d5:         0,
                    alpha5: -pi/2.,  a5:         0,  d6:         0,
                    alpha6: 0, a6: 0, d7: 0.200, q7: pi}
        T0_1 = get_tf_from_dh(alpha0, a0, d1, q1).subs(dh_parameters)
        T1_2 = get_tf_from_dh(alpha1, a1, d2, q2).subs(dh_parameters)
        T2_3 = get_tf_from_dh(alpha2, a2, d3, q3).subs(dh_parameters)
        T3_4 = get_tf_from_dh(alpha3, a3, d4, q4).subs(dh_parameters)
        T4_5 = get_tf_from_dh(alpha4, a4, d5, q5).subs(dh_parameters)
        T5_6 = get_tf_from_dh(alpha5, a5, d6, q6).subs(dh_parameters)
        T6_EE = get_tf_from_dh(alpha6, a6, d7, q7).subs(dh_parameters)
        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

        return T0_EE
        
    def _robot_FK_3(self):
        q1, q2, q3 = symbols('q1:4')
        d1, d2, d3 = symbols('d1:4')
        a0, a1, a2 = symbols('a0:3')
        alpha0, alpha1, alpha2 = symbols('alpha0:3')
        dh_parameters = {alpha0:        0,  a0:         0,  d1:      0.780,
                    alpha1: -pi/2.,  a1:      0.320,  d2:         0, q2: q2 - pi/2.,
                    alpha2:        0,  a2:      1.280,  d3:         0}
        T0_1 = get_tf_from_dh(alpha0, a0, d1, q1).subs(dh_parameters)
        T1_2 = get_tf_from_dh(alpha1, a1, d2, q2).subs(dh_parameters)
        T2_3 = get_tf_from_dh(alpha2, a2, d3, q3).subs(dh_parameters)
        T0_EE = T0_1 * T1_2 * T2_3

        return T0_EE  

    # def forward_kinematics(self, q) -> List[float]:
    #     q_symbols = symbols('q0:8')
    #     subs = {q_symbol: qi for q_symbol, qi in zip(q_symbols, q)}
    #     print(subs)
    #     T0_EE = self.robot_FK.subs(subs).evalf()
    #     T_B_F = self.gantry_FK.subs(subs).evalf()
    #     T_F_0 = get_tf_fixed_joint(self._robot.joints[2])
    #     TB_EE = T_B_F @ T_F_0 @ T0_EE
    #     position = TB_EE[:3, 3]
    #     quat = R.from_matrix(TB_EE[:3, :3]).as_quat()
    #     return position, quat

    def forward_kinematics(self, q) -> List[float]:
        q_symbols = symbols('q1:8')
        subs = {q_symbol: qi for q_symbol, qi in zip(q_symbols, q)}
        T0_EE = self.robot_FK.subs(subs).evalf()
        position = list(T0_EE[:3, 3])
        quat = R.from_matrix(T0_EE[:3, :3]).as_quat()
        pose = [*position, *quat]
        return pose    


    def inverse_kinematics(self, pose: List[float]) -> Tuple[float]:
        d1 = 0.780
        a1 = 0.320
        a2 = 1.280
        a3 = 0.200
        d4 = 1.5925
        d7 = 0.200
        alpha3 = -pi/2.
        alpha4 = pi/2
        alpha5 = -pi/2
        q7 = pi

        position = pose[:3]
        quat = pose[3:]

        R_0_EE = R.from_quat(quat).as_matrix()
        
        wrist_center = position - d7 * R_0_EE[:, 2]
        theta1 = atan2(wrist_center[1], wrist_center[0])
        theta1 = theta1.evalf()
            
        side_a = sqrt(a3**2 + d4**2)
        proj_z = wrist_center[2] - d1
        proj_xy = sqrt(wrist_center[0]**2 + wrist_center[1]**2) - a1
        side_b = sqrt(proj_z**2 + proj_xy**2)
        side_c = a2
        angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a)/(2 * side_b * side_c))
        angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b)/(2 * side_a * side_c))

        theta2 = pi/2. - angle_a - atan2(proj_z, proj_xy)
        theta2 = theta2.evalf()
        theta3 = pi - angle_b - atan2(d4, a3)
        theta3 = theta3.evalf()

        q1, q2, q3 = symbols('q1:4')
        
        subs = {q1: theta1, q2: theta2, q3: theta3}
        T_0_W = self.robot_FK_3.subs(subs).evalf()
        R_0_W = T_0_W[:3, :3]

        alpha = alpha3 + alpha4 + alpha5
        R_CORR = Matrix(R.from_euler("XYZ", [alpha, 0, q7]).as_matrix())

        R_W_EE = R_0_W.inv() @ R_0_EE @ R_CORR.inv()

        theta4, theta5, theta6 = R.from_matrix(R_W_EE.evalf()).as_euler("YZY")

        thetas = [theta1, theta2, theta3, theta4, theta5, theta6]

        return thetas




def main():
    urdf_file = "/home/workspace/robot-urdfs/combined.urdf"

    ik_solver = IKSolver(urdf_file)

    pose = [2.112, 0.000, 2.261, 0.000, 0.707, 0.000, 0.707]
    q = ik_solver.inverse_kinematics(pose)
    pose_solution = ik_solver.forward_kinematics(q)

    pose_np = np.array(pose).astype(float)
    pose_solution_np = np.array(pose_solution).astype(float)

    print(np.allclose(pose_np, pose_solution_np, atol=1e-3))


if __name__ == '__main__':
    main()