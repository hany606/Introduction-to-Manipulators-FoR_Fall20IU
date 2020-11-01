# - Both FK and IK should be implemented as distinct files.
# - IK function should take into account singularities, workspace limits and
# possibility of multiple solutions.
import numpy as np

def IK(end_effector):
    q = []   # generalized coordinates
    return q