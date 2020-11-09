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

    def _get_jacobian_column(self, dT):
        J = np.zeros((6,1))
        
        J[:3, 0] = dT[:3,3]
        J[3,0] = dT[2,1]
        J[4,0] = dT[0,2]
        J[5,0] = dT[1,0]
        return J.squeeze()

    def calc_numerical(self, q):
        J = np.zeros((6,6))
        # FK Zero configuration
        T = self.T_base_robot @ rotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]) @ translation_x(self.l[2]) @ rotation_y(q[2]) @ translation_x(self.l[3]) @ rotation_x(q[3]) @ translation_x(self.l[4]) @ rotation_y(q[4]) @ rotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot

        To_inv = np.eye(4)
        To_inv[:3,:3] = np.linalg.inv(T[:3,:3])

        dT = self.T_base_robot @ drotation_z(q[0]) @ translation_z(self.l[0]) @ translation_x(self.l[1]) @ rotation_y(q[1]) @ translation_x(self.l[2]) @ rotation_y(q[2]) @ translation_x(self.l[3]) @ rotation_x(q[3]) @ translation_x(self.l[4]) @ rotation_y(q[4]) @ rotation_x(q[5]) @ translation_x(self.l[5]) @ self.T_tool_robot @ To_inv
        # print(dT)
        J[:,0] = self._get_jacobian_column(dT)
        # print(J[:, 0])
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
        T0i = np.eye(4)
        for i in range(6):
            T0i = T0i @ A[i]
            O.append(T0i[:3,3])
            U.append(T0i[:3, u_rotation_joints_cols[i]])
        T0i = T0i @ A[6]
        O.append(T0i[:3,3])
        J = np.zeros((6,6))
        for i in range(6):
            J[:3,i] = np.cross((U[i]).reshape((1,3)), (O[6] - O[i]).reshape((1,3))).T.squeeze()
            J[3:,i] = U[i]
        return J

    def calc_sympolic(self, q):
        # Just to know which method works fine from the first three elements of each jacobian -> it appeared that the numerical derivatives has a problem
        q = q.squeeze()
        # use sympy to calculate symbolically
        def sp_translation_x(l):
            return np.array([[1,0,0, l],
                            [0,1,0, 0],
                            [0,0,1, 0],
                            [0,0,0, 1]])

        def sp_translation_y(l):
            return np.array([[1,0,0, 0],
                            [0,1,0, l],
                            [0,0,1, 0],
                            [0,0,0, 1]])

        def sp_translation_z(l):
            return np.array([[1,0,0, 0],
                            [0,1,0, 0],
                            [0,0,1, l],
                            [0,0,0, 1]])

        def sp_rotation_x(theta):
            return np.array([[1,         0,          0, 0],
                            [0,sp.cos(theta),-sp.sin(theta), 0],
                            [0,sp.sin(theta), sp.cos(theta), 0],
                            [0,         0,          0, 1]])


        def sp_rotation_y(theta):
            return np.array([[sp.cos(theta) ,0,sp.sin(theta), 0],
                            [0          ,1,         0, 0],
                            [-sp.sin(theta),0,sp.cos(theta), 0],
                            [0          ,0,         0, 1]])

        def sp_rotation_z(theta):
            return np.array([[sp.cos(theta),-sp.sin(theta),0, 0],
                            [sp.sin(theta), sp.cos(theta),0, 0],
                            [0         ,0          ,1, 0],
                            [0         ,0          ,0, 1]])

        q0, q1, q2, q3, q4, q5 = sp.symbols("q0 q1 q2 q3 q4 q5", real=True)
        l0, l1, l2, l3, l4, l5 = sp.symbols("l0 l1 l2 l3 l4 l5", real=True)
        mat = sp.Matrix

        T = mat(self.T_base_robot) @ mat(sp_rotation_z(q0)) @ mat(sp_translation_z(l0)) @ mat(sp_translation_x(l1)) @ mat(sp_rotation_y(q1)) @ mat(sp_translation_x(l2)) @ mat(sp_rotation_y(q2)) @ mat(sp_translation_x(l3)) @ mat(sp_rotation_x(q3)) @ mat(sp_translation_x(l4)) @ mat(sp_rotation_y(q4)) @ mat(sp_rotation_x(q5)) @ mat(sp_translation_x(l5)) @ mat(self.T_tool_robot)

        J = np.zeros((6,6))
        for i in range(3):
            J[i,0] = T[i,3].diff(q0).subs({q0:q[0], q1:q[1], q2:q[2], q3:q[3], q4:q[4], q5:q[5],
                                    l0:self.l[0], l1:self.l[1], l2:self.l[2], l3:self.l[3], l4:self.l[4], l5:self.l[5]})
            J[i,1] = T[i,3].diff(q1).subs({q0:q[0], q1:q[1], q2:q[2], q3:q[3], q4:q[4], q5:q[5],
                                    l0:self.l[0], l1:self.l[1], l2:self.l[2], l3:self.l[3], l4:self.l[4], l5:self.l[5]})        
            J[i,2] = T[i,3].diff(q2).subs({q0:q[0], q1:q[1], q2:q[2], q3:q[3], q4:q[4], q5:q[5],
                                    l0:self.l[0], l1:self.l[1], l2:self.l[2], l3:self.l[3], l4:self.l[4], l5:self.l[5]})
            J[i,3] = T[i,3].diff(q3).subs({q0:q[0], q1:q[1], q2:q[2], q3:q[3], q4:q[4], q5:q[5],
                                    l0:self.l[0], l1:self.l[1], l2:self.l[2], l3:self.l[3], l4:self.l[4], l5:self.l[5]})
            J[i,4] = T[i,3].diff(q4).subs({q0:q[0], q1:q[1], q2:q[2], q3:q[3], q4:q[4], q5:q[5],
                                    l0:self.l[0], l1:self.l[1], l2:self.l[2], l3:self.l[3], l4:self.l[4], l5:self.l[5]})
            J[i,5] = T[i,3].diff(q5).subs({q0:q[0], q1:q[1], q2:q[2], q3:q[3], q4:q[4], q5:q[5],
                                    l0:self.l[0], l1:self.l[1], l2:self.l[2], l3:self.l[3], l4:self.l[4], l5:self.l[5]})
        # print(J)
        return J

        
if __name__ == "__main__":
    jacobian = Jacobian()

    q = np.zeros((6,1))

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