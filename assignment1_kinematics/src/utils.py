import numpy as np
from math import sin, cos

def translation_x(l):
    return np.array([[1,0,0, l],
                     [0,1,0, 0],
                     [0,0,1, 0],
                     [0,0,0, 1]])

def translation_y(l):
    return np.array([[1,0,0, 0],
                     [0,1,0, l],
                     [0,0,1, 0],
                     [0,0,0, 1]])

def translation_z(l):
    return np.array([[1,0,0, 0],
                     [0,1,0, 0],
                     [0,0,1, l],
                     [0,0,0, 1]])

def rotation_x(theta):
    return np.array([[1,         0,          0, 0],
                     [0,cos(theta),-sin(theta), 0],
                     [0,sin(theta), cos(theta), 0],
                     [0,         0,          0, 1]])


def rotation_y(theta):
    return np.array([[cos(theta) ,0,sin(theta), 0],
                     [0          ,1,         0, 0],
                     [-sin(theta),0,cos(theta), 0],
                     [0          ,0,         0, 1]])

def rotation_z(theta):
    return np.array([[cos(theta),-sin(theta),0, 0],
                     [sin(theta), cos(theta),0, 0],
                     [0         ,0          ,1, 0],
                     [0         ,0          ,0, 1]])
