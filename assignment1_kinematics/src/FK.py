# - Both FK and IK should be implemented as distinct files.

import numpy as np

# q -- generalized coordinates
def FK(q):
    frames = [[],[]]
    end_effoctor = []
    return frames, end_effoctor