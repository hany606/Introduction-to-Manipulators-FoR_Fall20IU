# This file is made in such to be able to used independently --> constrain from the assignment
# Also, to be able to be used inside the class
# - Both FK and IK should be implemented as distinct files.
# - IK function should take into account singularities, workspace limits and
# possibility of multiple solutions.
import numpy as np
from robot import KUKA_KR10_R1100_2_configs as configs
from utils import *
from scipy.optimize import fsolve


def q0_solver(x, y, l1, q0_init):
    return [y*np.sin(q0_init)-l1-x*np.cos(q0_init)]

def IK(T, T_base=None, T_tool=None, m=-1, debug=True):
    status = "q1, q2, q3 (Manipulator part):"
    inv = np.linalg.inv
    l = configs.get_links_dimensions()
    joint_limits = configs.get_joints_limits()
    q = [0 for i in range(6)]   # generalized coordinates
    
    T_base = translation_x(0) if T_base is None else T_base
    T_tool = translation_x(0) if T_tool is None else T_tool
    T_o = inv(translation_z(l[0])) @ inv(T_base) @ T @ inv(T_tool) @ inv(translation_z(l[3])) #@ inv(translation_x(l[3]+l[4]))
    # print("####### DEBUG")

    position = get_position(T_o)
    x, y, z = position[0], position[1], position[2]
    # print(position)


    # TODO: Deal with singularities and exceptions
    q[0] = q0_solver(x, y, l[0], 0.1)[0]
    if(np.cos(q[0]) != 0):
        q[1] = np.arctan2(y+l[0]*np.sin(q[0]), np.cos(q[0])*z)
    else:
        q[1] = np.arctan2(y+l[0]*np.sin(q[0]), z)

    if(np.cos(q[1]) != 0):
        q[2] = z/np.cos(q[1]) - l[2]
    else:
        q[2] = z - l[2]
    status += "\nq4, q5, q6 (Wrist part):"
    T_123 = rotation_z(q[0]) @ translation_x(-l[1]) @ rotation_x(q[1]) @ translation_z(l[2]) @ translation_z(q[2])
    # print((inv(T_123) @ T_o))


    orientation = get_rotation((inv(T_123) @ T_o))
    n, s, a = orientation[:,0], orientation[:,1], orientation[:,2]

    # Check condition of singularity for q[4] -> q_5 (The signle term in the matrix)
    singularity_condition2 = abs(a[2])
    # Singularity
    if(singularity_condition2 == 1):
        q[4] = np.arccos(a[2])
        # Here it is singularity such that q_4 + 4_6 = number
        # We need to take into account the joint limits
        # How to choose q_4 -> q[3] and q_6 -> q[5]: because the last joint is the one with biggest values, we will choose to move it to its limits
        # To improve this method, it is better to have a history for the previous movement of the robot and choose the angle that is close to each joint
        # Instead of the situation that 6th joint in its minimum and we need to reach its maximum but if we moved 4th joint we will get that orientation easily, but in the implemented method, it will not work like that
        angle = np.arctan2(n[1], n[0])
        q[5] = max(min(angle,joint_limits[5][1]), joint_limits[5][0])
        q[3] = angle - q[5]
        status += "\nMany solutions [Gimbal-lock] (q_4+q_6=angle), however, only one solution has been selected"
    else:
        q[3] = np.arctan2(a[0], -a[1])
        q[5] = np.arctan2(n[2], s[2])
        singularity_condition3 = n[2]
        if(singularity_condition3 == 0):
            if(np.cos(q[3]) == 0):
                q[4] = np.arctan2(a[1],0)
            else:
                q[4] = np.arctan2(-(a[1]/np.cos(q[3])),a[2])
        else:
            if(np.sin(q[5]) == 0):
                q[4] = np.arctan2(n[2], 0)
            else:
                q[4] = np.arctan2((n[2]/np.sin(q[5])), a[2])
        status += "\nOne Solution"

    

    return q, status