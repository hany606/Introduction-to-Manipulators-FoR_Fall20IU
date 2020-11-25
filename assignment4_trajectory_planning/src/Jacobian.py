# This file to calculate the jacobian with two methods: Skew theory & Numberical derivatives
# It is made according to the requirement of the assignment to make it in seperate file
import numpy as np
from robot import RRR_robot_configs as configs
from utils import *
import sympy as sp

class Jacobian:
    def __init__(self, T_base=None, T_tool=None):
        self.T_base_robot = translation_x(0) if T_base is None else T_base
        self.T_tool_robot = translation_x(0) if T_tool is None else T_tool
        self.l = configs.get_links_dimensions()

    def _get_jacobian_column(self, dT):
        J = np.zeros((6,1))
        
        J[:3, 0] = dT[:3,3]
        J[3,0] = dT[2,1]
        J[4,0] = dT[0,2]
        J[5,0] = dT[1,0]
        return J.squeeze()

    def calc_numerical(self, q):
        J = np.zeros((6,3))
        # FK Zero configuration
        T = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ rotation_y(q[1]) @ translation_x(self.l[1]) @ rotation_y(q[2]) @ translation_x(self.l[2]) @ self.T_tool_robot
    
        To_inv = np.eye(4)
        To_inv[:3,:3] = np.linalg.inv(T[:3,:3])

        dT = self.T_base_robot @ drotation_z(q[0]) @ translation_z(self.l[0]) @ rotation_y(q[1]) @ translation_x(self.l[1]) @ rotation_y(q[2]) @ translation_x(self.l[2]) @ self.T_tool_robot @ To_inv
        J[:,0] = self._get_jacobian_column(dT)

        dT = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ drotation_y(q[1]) @ translation_x(self.l[1]) @ rotation_y(q[2]) @ translation_x(self.l[2]) @ self.T_tool_robot @ To_inv
        J[:,1] = self._get_jacobian_column(dT)

        dT = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ rotation_y(q[1]) @ translation_x(self.l[1]) @ drotation_y(q[2]) @ translation_x(self.l[2]) @ self.T_tool_robot @ To_inv
        J[:,2] = self._get_jacobian_column(dT)

        return J
        
if __name__ == "__main__":
    jacobian = Jacobian()

    q = np.zeros((3,1))

    q[1] = np.pi/4
    skew = jacobian.calc_skew(q)
    numerical = jacobian.calc_numerical(q)
    print("Skew:")
    print(skew)
    print("----------------")
    print("Numerical Derivatives:")
    print(numerical)
    print("----------------")

    print(calc_error(numerical, skew))

    # symbolic = jacobian.calc_sympolic(q)
    # print("Symbolic")
    # print(symbolic)