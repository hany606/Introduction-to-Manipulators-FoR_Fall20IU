# This file to calculate the jacobian with two methods: Skew theory & Numberical derivatives
# It is made according to the requirement of the assignment to make it in seperate file
import numpy as np
from robot import KUKA_KR10_R1100_2_configs as configs
from utils import *

class Jacobian:
    def __init__(self, T_base=None, T_tool=None):
        self.T_base_robot = translation_x(0) if T_base is None else T_base
        self.T_tool_robot = translation_x(0) if T_tool is None else T_tool
        self.l = configs.get_links_dimensions()

    def _get_jacobian_column(self, dT):
        J = np.zeros((6,1))

        J[:3, 0] = dT[:3,3]
        J[3,0] = dT[2,1]
        J[4,0] = dT[0,3]
        J[5,0] = dT[1,0]
        return J.squeeze()

    def calc_numerical(self, q):
        # FK Zero configuration
        T = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]) @ translation_x(self.l[2]) @ rotation_y(q[2]) @ translation_x(self.l[3]) @ rotation_x(q[3]) @ translation_x(self.l[4]) @ rotation_y(q[4]) @ rotation_x(q[5]) @ self.T_tool_robot

        To = T
        To[:3, 3] = np.zeros((3,))
        J = np.zeros((6,6))

        To_inv = To.T

        # dT = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]) @ translation_x(self.l[2]) @ rotation_y(q[2]) @ translation_x(self.l[3]) @ rotation_x(q[3]) @ translation_x(self.l[4]) @ rotation_y(q[4]) @ rotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot @ To_inv

        dT = self.T_base_robot @ drotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]) @ translation_x(self.l[2]) @ rotation_y(q[2]) @ translation_x(self.l[3]) @ rotation_x(q[3]) @ translation_x(self.l[4]) @ rotation_y(q[4]) @ rotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot @ To_inv
        J[:,0] = self._get_jacobian_column(dT)

        dT = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ drotation_y(q[1]) @ translation_x(self.l[2]) @ rotation_y(q[2]) @ translation_x(self.l[3]) @ rotation_x(q[3]) @ translation_x(self.l[4]) @ rotation_y(q[4]) @ rotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot @ To_inv
        J[:,1] = self._get_jacobian_column(dT)

        dT = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]) @ translation_x(self.l[2]) @ drotation_y(q[2]) @ translation_x(self.l[3]) @ rotation_x(q[3]) @ translation_x(self.l[4]) @ rotation_y(q[4]) @ rotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot @ To_inv
        J[:,2] = self._get_jacobian_column(dT)

        dT = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]) @ translation_x(self.l[2]) @ rotation_y(q[2]) @ translation_x(self.l[3]) @ drotation_x(q[3]) @ translation_x(self.l[4]) @ rotation_y(q[4]) @ rotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot @ To_inv
        J[:,3] = self._get_jacobian_column(dT)

        dT = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]) @ translation_x(self.l[2]) @ rotation_y(q[2]) @ translation_x(self.l[3]) @ rotation_x(q[3]) @ translation_x(self.l[4]) @ drotation_y(q[4]) @ rotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot @ To_inv
        J[:,4] = self._get_jacobian_column(dT)

        dT = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]) @ translation_x(self.l[2]) @ rotation_y(q[2]) @ translation_x(self.l[3]) @ rotation_x(q[3]) @ translation_x(self.l[4]) @ rotation_y(q[4]) @ drotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot @ To_inv
        J[:,5] = self._get_jacobian_column(dT)
        
        return J

    def calc_skew(self, q):
        # frames_transitions =  [ self.T_base_robot,
        #                         rotation_z(q[0]),
        #                         translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]),
        #                         translation_x(self.l[2]) @ rotation_y(q[2]),
        #                         translation_x(self.l[3]) @ rotation_x(q[3]),
        #                         translation_x(self.l[4]) @ rotation_y(q[4]),
        #                         rotation_x(q[5]),
        #                         self.T_tool_robot]
        A =  [  self.T_base_robot,
                rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]),
                rotation_y(q[1]) @ translation_x(self.l[2]),
                rotation_y(q[2]) @ translation_x(self.l[3]),
                rotation_x(q[3]) @ translation_x(self.l[4]),
                rotation_y(q[4]),
                rotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot]
        # calculate O, U vectors
        O = []
        U = []
        # z y y x y x
        u_rotation_joints_cols = [2, 1, 1, 0, 1, 0]
        for i in range(7):
            A0i = np.eye(4)
            for Ai in A[:i+1]: 
                A0i = A0i @ Ai
            # print(A0i.shape)
            O.append(A0i[:3,3])
            U.append(A0i[:3, u_rotation_joints_cols[i-1]])
        J = np.zeros((6,6))
        for i in range(6):
            J[:3,i] = np.cross((U[i]).reshape((1,3)), (O[6] - O[i]).reshape((1,3))).T.squeeze()
            J[3:,i] = U[i]

        return J


if __name__ == "__main__":
    jacobian = Jacobian()

    q = np.zeros((6,1))

    numerical = jacobian.calc_numerical(q)
    skew = jacobian.calc_skew(q)
    print(skew)
    print(numerical)

    print(calc_error(numerical, skew))