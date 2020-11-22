# This file is made in such to be able to used independently --> constrain from the assignment
# Also, to be able to be used inside the class
# - Both FK and IK should be implemented as distinct files.
import numpy as np
from robot import KUKA_KR10_R1100_2_configs as configs
from utils import *

# q -- generalized coordinates (thetas)
# Input theta vector, get end effector position
def FK(q, T_base=None, T_tool=None, return_frames=True):
    l = configs.get_links_dimensions()
    T_base_robot = translation_x(0) if T_base is None else T_base
    T_tool_robot = translation_x(0) if T_tool is None else T_tool
    
    # Not zero configuration
    # frames_transitions = [translation_x(0),
    #                       rotation_z(q[0]),
    #                       translation_z(l[0]) @ translation_x(l[1]) @ rotation_y(q[1]),
    #                       translation_z(l[2]) @ rotation_y(q[2]),
    #                       translation_z(l[3]) @ translation_x(l[4]) @ rotation_x(q[3]),
    #                       rotation_y(q[4]),
    #                       rotation_x(q[5])]

    # Zero configuration TODO: Change
    frames_transitions =  [ T_base_robot,
                            rotation_z(q[0]) @ translation_z(l[0]) @ translation_x(-l[1]),
                            rotation_x(q[1]) @ translation_z(l[2]),
                            translation_z(q[2]),
                            rotation_z(q[3]),
                            rotation_x(q[4]),
                            rotation_x(q[5]) @ translation_z(l[3]) @ T_tool_robot]
    
    T0i = np.eye(4,4)
    frames = []
    for i in range(7):
        T0i = T0i @ frames_transitions[i]

        frames.append(T0i)

    # print(len(frames))
    if(return_frames == True):
        return frames
    return frames[-1]

if __name__ == "__main__":
    # print(configs.get_links_dimensions())
    q = np.zeros((6,))
    q[1] = np.pi/2
    frames = FK(q)
    for f in frames:
        print(f)
        print("-----------")
    print(get_rotation(frames[-1]), get_position(frames[-1]))
