# This file to calculate the jacobian with two methods: Skew theory & Numberical derivatives
# It is made according to the requirement of the assignment to make it in seperate file
import numpy as np
from robot import FANUC_R_2000i_configs as configs
# from utils import *
from utils import translation_x as tx
from utils import translation_y as ty
from utils import translation_z as tz

from utils import rotation_x as rx
from utils import rotation_y as ry
from utils import rotation_z as rz

from utils import dtranslation_x as dtx
from utils import dtranslation_y as dty
from utils import dtranslation_z as dtz

from utils import drotation_x as drx
from utils import drotation_y as dry
from utils import drotation_z as drz

class Jacobian:
    def __init__(self, robot_configs, T_base=None, T_tool=None):
        self.T_base = tx(0) if T_base is None else T_base
        self.T_tool = tx(0) if T_tool is None else T_tool
        self.d = robot_configs.get_links_dimensions()

    def _get_jacobian_column(self, dT):
        J = np.zeros((6,1))
        
        J[:3, 0] = dT[:3,3]
        J[3,0] = dT[2,1]
        J[4,0] = dT[0,2]
        J[5,0] = dT[1,0]
        return J.squeeze()

    def calc_identification_jacobian(self, T_base, T_tool, q, pi, pi_0, num_unknown_parameters=18):
        J = np.zeros((6,num_unknown_parameters))
        # Reducible Kinmatic Model: pi_0 is the nomial pi for the unknown parameters
        T = T_base @ rz(q[0]) @ tx(self.d[1]+pi[0]) @ ty(pi[1]) @ rx(pi[2]) @ ry(q[1]+pi[3]) @ tx(pi[4]) @ rx(pi[5]) @ rz(pi[6]) @ ry(q[2]+pi[7]) @ tx(self.d[5]+pi[8]) @ tz(self.d[4]+pi[9]) @ rz(pi[10]) @ rx(q[3]+pi[11]) @ ty(pi[12]) @ tz(pi[13]) @ rz(pi[14]) @ ry(q[4] + pi[15]) @ tz(pi[16]) @ rz(pi[17]) @ rx(q[5]) @ T_tool
        
        To_inv = np.eye(4)
        To_inv[:3,:3] = np.linalg.inv(T[:3,:3])
        
        dT = T_base @ rz(q[0]) @ dtx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,0] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ dty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,1] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ drx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,2] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ dry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,3] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ dtx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,4] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ drx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,5] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ drz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,6] = self._get_jacobian_column(dT)
        
        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ dry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,7] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ dtx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,8] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ dtz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,9] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ drz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,10] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ drx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,11] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ dty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,12] = self._get_jacobian_column(dT)       

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ dtz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,13] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ drz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,14] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ dry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,15] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ dtz(pi_0[16]) @ rz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,16] = self._get_jacobian_column(dT)

        dT = T_base @ rz(q[0]) @ tx(self.d[1]+pi_0[0]) @ ty(pi_0[1]) @ rx(pi_0[2]) @ ry(q[1]+pi_0[3]) @ tx(pi_0[4]) @ rx(pi_0[5]) @ rz(pi_0[6]) @ ry(q[2]+pi_0[7]) @ tx(self.d[5]+pi_0[8]) @ tz(self.d[4]+pi_0[9]) @ rz(pi_0[10]) @ rx(q[3]+pi_0[11]) @ ty(pi_0[12]) @ tz(pi_0[13]) @ rz(pi_0[14]) @ ry(q[4] + pi_0[15]) @ tz(pi_0[16]) @ drz(pi_0[17]) @ rx(q[5]) @ T_tool @ To_inv
        J[:,17] = self._get_jacobian_column(dT)

        return J
        
if __name__ == "__main__":
    from robot import FANUC_R_2000i_configs
    jacobian = Jacobian(robot_configs=FANUC_R_2000i_configs)

    q = np.zeros((6,1))

    q[1] = np.pi_0/4
    numerical = jacobian.calc_numerical(q)
    print("Numerical Derivatives:")
    print(numerical)
    print("----------------")