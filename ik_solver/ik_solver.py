from __future__ import annotations

from urdf_parser_py import urdf
from typing import List
from pathlib import Path
from sympy import *
from scipy.spatial.transform import Rotation as R
from utils import get_tf_fixed_joint, get_tf_from_dh


class IKSolver:
    def __init__(self, robot_urdf: Path):
        self._robot = urdf.Robot.from_xml_file(robot_urdf)
        self.active_joints = [
            joint for joint in self._robot.joints if joint != "fixed"]
        self.dof = len(self.active_joints)
        self.robot_FK = self._robot_FK()
        self.gantry_FK = self._gantry_FK()
        
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
        

    def forward_kinematics(self, q) -> List[float]:
        q_symbols = symbols('q0:8')
        subs = {q_symbol: qi for q_symbol, qi in zip(q_symbols, q)}
        print(subs)
        T0_EE = self.robot_FK.subs(subs).evalf()
        T_B_F = self.gantry_FK.subs(subs).evalf()
        T_F_0 = get_tf_fixed_joint(self._robot.joints[2])
        TB_EE = T_B_F @ T_F_0 @ T0_EE
        position = TB_EE[:3, 3]
        quat = R.from_matrix(TB_EE[:3, :3]).as_quat()
        return position, quat
        

    def inverse_kinematics(self, x: List[float]) -> List[float]:
        pass

def main():
    urdf_file = "/home/workspace/robot-urdfs/combined.urdf"

    ik_solver = IKSolver(urdf_file)
    q = [6.707101, 0.6076538323743459, -0.9776461805046237, -0.20725784867432662,4.566828520768364, -1.4761645747517642, -0.40463713378236577]
    position, quat = ik_solver.forward_kinematics(q)
    print(position, quat)

if __name__ == '__main__':
    main()