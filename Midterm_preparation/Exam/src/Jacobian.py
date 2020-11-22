# This file to calculate the jacobian with two methods: Skew theory & Numberical derivatives
# It is made according to the requirement of the assignment to make it in seperate file
import numpy as np
from robot import KUKA_KR10_R1100_2_configs as configs
from utils import *
import sympy as sp

class Jacobian:
    def __init__(self, T_base=None, T_tool=None):
        self.T_base_robot = translation_x(0) if T_base is None else T_base
        self.T_tool_robot = translation_x(0) if T_tool is None else T_tool
        self.l = configs.get_links_dimensions()
    
    def calc_skew(self, q):
        A =  [ self.T_base_robot,
                            rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(-self.l[1]),
                            rotation_x(q[1]) @ translation_z(self.l[2]),
                            translation_z(q[2]),
                            rotation_z(q[3]),
                            rotation_x(q[4]),
                            rotation_x(q[5]) @ translation_z(self.l[3]) @ self.T_tool_robot]
        # calculate O, U vectors
        O = []
        U = []
        # z y y x y x
        u_rotation_joints_cols = [2, 0, 2, 2, 0, 2]
        T0i = np.eye(4)
        for i in range(6):
            T0i = T0i @ A[i]
            O.append(T0i[:3,3])
            U.append(T0i[:3, u_rotation_joints_cols[i]])
        T0i = T0i @ A[6]
        O.append(T0i[:3,3])
        J = np.zeros((6,6))
        for i in range(6):
            if(i == 2):
                J[:3,i] = U[i]
                J[3:,i] = np.zeros((3,))
                continue    
            J[:3,i] = np.cross((U[i]).reshape((1,3)), (O[6] - O[i]).reshape((1,3))).T.squeeze()
            J[3:,i] = U[i]
        return J


        
if __name__ == "__main__":
    jacobian = Jacobian()

    q = np.zeros((6,1))

    q[1] = np.pi/4
    skew = jacobian.calc_skew(q)
    print("Skew:")
    print(skew)
    print("----------------")