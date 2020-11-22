from scipy.optimize import fsolve


import math
import numpy as np

def func(x, y, l1, q0_init):
    x = 1
    y = 1
    l1 = 5 
    return [y*np.sin(phi[0])-l1-x*np.cos(phi[0])]

phi = fsolve(func,(0))

print(phi)
