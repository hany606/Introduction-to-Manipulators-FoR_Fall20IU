# This file is made in such to be able to used independently --> constrain from the assignment
# Also, to be able to be used inside the class
# - Both FK and IK should be implemented as distinct files.
# - IK function should take into account singularities, workspace limits and
# possibility of multiple solutions.
import numpy as np
from robot import RRR_robot_configs as configs
from utils import *


def IK(T, T_base=None, T_tool=None, m=-1, debug=True):
    status = "q1, q2, q3 (Manipulator part):"
    inv = np.linalg.inv
    l = configs.get_links_dimensions()
    joint_limits = configs.get_joints_limits()
    q = [0 for i in range(3)]   # generalized coordinates
    
    T_base = translation_x(0) if T_base is None else T_base
    T_tool = translation_x(0) if T_tool is None else T_tool
    # TODO: Change
    T_o = inv(translation_z(l[0])) @ inv(T_base) @ T @ inv(T_tool)

    position = get_position(T_o)
    x, y, z = position[0], position[1], position[2]
    # print(position)

    x_dash = np.sqrt(x**2+y**2) # positive and negative on the part of square root
    y_dash = -z
    l1_dash = l[1]
    l2_dash = l[2]
    
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
    singularity_condition1 = l[1]*np.cos(q[1]) + (l[2])*np.cos(q[1]+q[2])
    if(singularity_condition1 == 0):
        # To improve this method, it is better to have a history for the previous movement of the robot, in order to, keep the value of the joint as it is
        q[0] = 0 # any -- it is a point that positioned on z-axis
        status += "\nMany Solutions [rotation of q_1] (q1 = any) -- on z-axis"
    else:
        q[0] = np.arctan2(y,x)
        status += "\nOne Solution"

    return q, status

    