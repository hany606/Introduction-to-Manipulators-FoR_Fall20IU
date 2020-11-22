# This file is made in such to be able to used independently --> constrain from the assignment
# Also, to be able to be used inside the class
# - Both FK and IK should be implemented as distinct files.
# - IK function should take into account singularities, workspace limits and
# possibility of multiple solutions.
import numpy as np
from robot import KUKA_KR10_R1100_2_configs as configs
from utils import *


def IK(T, T_base=None, T_tool=None, m=-1, debug=True):
    status = "q1, q2, q3 (Manipulator part):"
    inv = np.linalg.inv
    l = configs.get_links_dimensions()
    joint_limits = configs.get_joints_limits()
    q = [0 for i in range(6)]   # generalized coordinates
    
    T_base = translation_x(0) if T_base is None else T_base
    T_tool = translation_x(0) if T_tool is None else T_tool
    # TODO: Change
    T_o = inv(translation_z(l[0])) @ inv(T_base) @ T @ inv(T_tool) @ inv(translation_x(l[5])) #@ inv(translation_x(l[3]+l[4]))
    print("####### DEBUG")
    print_matrix(T_o)
    print("####### DEBUG")

    position = get_position(T_o)
    x, y, z = position[0], position[1], position[2]
    # print(position)

    x_dash = np.sqrt(x**2+y**2) - l[1] # positive and negative on the part of square root
    y_dash = -z
    l1_dash = l[2]
    l2_dash = l[3]+l[4]
    
    # Get q2 = q[1], q3 = q[2]
    # print("####### DEBUG")
    # print((x_dash**2+y_dash**2-l1_dash**2-l2_dash**2))
    # print((x_dash**2+y_dash**2-l1_dash**2-l2_dash**2)/(2*l1_dash*l2_dash))
    # The denominator is always positive, thus, the sign of the angle determined with the numerator
    # using np.round: as (x_dash**2+y_dash**2-l1_dash**2-l2_dash**2)/(2*l1_dash*l2_dash) makes some problems in range due to approximation, e.g. 1.0000000000000004 ~ 1
    q[2] = np.arccos(np.round(x_dash**2+y_dash**2-l1_dash**2-l2_dash**2,6)/(2*l1_dash*l2_dash))
    m = np.sign(q[2])
    q[1] = -m * np.arctan((l2_dash*np.sin(q[2]))/(l1_dash+l2_dash*np.cos(q[2]))) + np.arctan(y_dash/x_dash)
    # Check condition of singularity and get q1
    singularity_condition1 = l[1] + l[2]*np.cos(q[1]) + (l[3]+l[4])*np.cos(q[1]+q[2])
    if(singularity_condition1 == 0):
        # To improve this method, it is better to have a history for the previous movement of the robot, in order to, keep the value of the joint as it is
        q[0] = 0 # any -- it is a point that positioned on z-axis
        status += "\nMany Solutions [rotation of q_1] (q1 = any) -- on z-axis"
    else:
        q[0] = np.arctan2(y,x)
        status += "\nOne Solution"


    status += "\nq4, q5, q6 (Wrist part):"
    T_123 = rotation_z(q[0]) @ translation_x(l[1]) @ rotation_y(q[1]) @ translation_x(l[2]) @ rotation_y(q[2]) @ translation_x(l[3] + l[4])
    # print((inv(T_123) @ T_o))
    orientation = get_rotation((inv(T_123) @ T_o))
    n, s, a = orientation[:,0], orientation[:,1], orientation[:,2]

    # Check condition of singularity for q[4] -> q_5 (The signle term in the matrix)
    singularity_condition2 = abs(n[0])
    # Singularity
    if(singularity_condition2 == 1):
        q[4] = np.arccos(n[0])
        # Here it is singularity such that q_4 + 4_6 = number
        # We need to take into account the joint limits
        # How to choose q_4 -> q[3] and q_6 -> q[5]: because the last joint is the one with biggest values, we will choose to move it to its limits
        # To improve this method, it is better to have a history for the previous movement of the robot and choose the angle that is close to each joint
        # Instead of the situation that 6th joint in its minimum and we need to reach its maximum but if we moved 4th joint we will get that orientation easily, but in the implemented method, it will not work like that
        angle = np.arctan2(s[2], s[1])
        q[5] = max(min(angle,joint_limits[5][1]), joint_limits[5][0])
        q[3] = angle - q[5]
        status += "\nMany solutions [Gimbal-lock] (q_4+q_6=angle), however, only one solution has been selected"
    else:
        q[3] = np.arctan2(n[1], -n[2])
        q[5] = np.arctan2(s[0], a[0])
        singularity_condition3 = a[0]
        if(singularity_condition3 == 0):
            if(np.sin(q[5]) == 0):
                q[4] = np.arctan2(s[0],0)
            else:
                q[4] = np.arctan2((s[0]/np.sin(q[5])),n[0])
        else:
            if(np.cos(q[5]) == 0):
                q[4] = np.arctan2(a[0], 0)
            else:
                q[4] = np.arctan2((a[0]/np.cos(q[5])), n[0])
        status += "\nOne Solution"


    return q, status