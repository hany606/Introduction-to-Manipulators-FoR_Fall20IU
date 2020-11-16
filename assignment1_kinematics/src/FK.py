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

    # Zero configuration
    frames_transitions =  [ T_base_robot,
                            rotation_z(q[0]) @ translation_z(l[0]) @ translation_x(l[1]),
                            rotation_y(q[1]) @ translation_x(l[2]),
                            rotation_y(q[2]) @ translation_x(l[3]),
                            rotation_x(q[3]) @ translation_x(l[4]),
                            rotation_y(q[4]),
                            rotation_x(q[5]) @ translation_x(l[5]) @ T_tool_robot]
    
    T0i = np.eye(4,4)
    frames = []
    for i in range(7):
        T0i = T0i @ frames_transitions[i]

        frames.append(T0i)

    # print(len(frames))
    if(return_frames == True):
        return frames
    return frames[-1]

    # Old
    #  # Zero configuration
    # frames_transitions = [T_base_robot,
    #                       rotation_z(q[0]),
    #                       translation_z(l[0]) @ translation_x(l[1]) @ rotation_y(q[1]),
    #                       translation_x(l[2]) @ rotation_y(q[2]),
    #                       translation_x(l[3]) @ rotation_x(q[3]),
    #                       translation_x(l[4]) @ rotation_y(q[4]),
    #                       rotation_x(q[5])]
    
    # frames = [T_base_robot]    # the world frame
    # for i in range(1,7):
    #     old_frame = frames[i-1]
    #     new_frame = old_frame @ frames_transitions[i]
    #     frames.append(new_frame)

    # end_effoctor = frames[-1] @ translation_x(l[5])
    # frames.append(end_effoctor)
    # tool = frames[-1] @ T_tool_robot
    # frames.append(tool)

    # # print(len(frames))
    # if(return_frames == True):
    #     return frames
    # return tool


if __name__ == "__main__":
    # print(configs.get_links_dimensions())
    q = np.zeros((6,))
    q[1] = np.pi/2
    frames = FK(q)
    for f in frames:
        print(f)
        print("-----------")
    print(get_rotation(frames[-1]), get_position(frames[-1]))
