# - Both FK and IK should be implemented as distinct files.
import numpy as np
from robot import KUKA_KR10_R1100_2
from utils import *

robot = KUKA_KR10_R1100_2()

# q -- generalized coordinates (thetas)
def FK(q):
    l = robot.links
    frames_transitions = [translation_x(0),
                          rotation_z(q[0]),
                          translation_z(l[0]) @ translation_x(l[1]) @ rotation_y(q[1]),
                          translation_z(l[2]) @ rotation_y(q[2]),
                          translation_z(l[3]) @ translation_x(l[4]) @ rotation_x(q[3]),
                          rotation_y(q[4]),
                          rotation_x(q[5])]
    frames = [frames_transitions[0]]    # the world frame
    for i in range(1,7):
        old_frame = frames[i-1]
        new_frame = old_frame @ frames_transitions[i]
        frames.append(new_frame)

    end_effoctor = frames[-1] @ translation_x(0)
    frames.append(end_effoctor)
    print(len(frames))
    return frames

if __name__ == "__main__":
    q = np.zeros((6,))
    # q[1] = -np.pi
    frames = FK(q)
    for f in frames:
        print(f)
        print("-----------")
    # print(frames[-1].shape)
    vector = np.zeros((4,1))
    vector[3,0] = 1
    print(np.matmul(frames[-1], vector))
