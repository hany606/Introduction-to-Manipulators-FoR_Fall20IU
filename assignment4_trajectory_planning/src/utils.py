import numpy as np
from sympy import simplify

def translation_x(l):
    return np.array([[1,0,0, l],
                     [0,1,0, 0],
                     [0,0,1, 0],
                     [0,0,0, 1]], dtype='float')

def translation_y(l):
    return np.array([[1,0,0, 0],
                     [0,1,0, l],
                     [0,0,1, 0],
                     [0,0,0, 1]], dtype='float')

def translation_z(l):
    return np.array([[1,0,0, 0],
                     [0,1,0, 0],
                     [0,0,1, l],
                     [0,0,0, 1]], dtype='float')

def rotation_x(theta):
    return np.array([[1,         0,          0, 0],
                     [0,np.cos(theta),-np.sin(theta), 0],
                     [0,np.sin(theta), np.cos(theta), 0],
                     [0,         0,          0, 1]], dtype='float')


def rotation_y(theta):
    return np.array([[np.cos(theta) ,0,np.sin(theta), 0],
                     [0          ,1,         0, 0],
                     [-np.sin(theta),0,np.cos(theta), 0],
                     [0          ,0,         0, 1]], dtype='float')

def rotation_z(theta):
    return np.array([[np.cos(theta),-np.sin(theta),0, 0],
                     [np.sin(theta), np.cos(theta),0, 0],
                     [0         ,0          ,1, 0],
                     [0         ,0          ,0, 1]], dtype='float')


def dtranslation_x(l):
    return np.array([[1,0,0, 1],
                     [0,1,0, 0],
                     [0,0,1, 0],
                     [0,0,0, 1]], dtype='float')

def dtranslation_y(l):
    return np.array([[1,0,0, 0],
                     [0,1,0, 1],
                     [0,0,1, 0],
                     [0,0,0, 1]], dtype='float')

def dtranslation_z(l):
    return np.array([[1,0,0, 0],
                     [0,1,0, 0],
                     [0,0,1, 1],
                     [0,0,0, 1]], dtype='float')

def drotation_x(theta):
    return np.array([[0,         0,          0, 0],
                     [0,-np.sin(theta), -np.cos(theta), 0],
                     [0, np.cos(theta), -np.sin(theta), 0],
                     [0,         0,          0, 0]], dtype='float')


def drotation_y(theta):
    return np.array([[-np.sin(theta), 0,  np.cos(theta), 0],
                     [0          ,0,         0, 0],
                     [-np.cos(theta), 0, -np.sin(theta), 0],
                     [0          ,0,         0, 0]], dtype='float')

def drotation_z(theta):
    return np.array([[-np.sin(theta),-np.cos(theta),0, 0],
                     [ np.cos(theta), -np.sin(theta),0, 0],
                     [0         ,0          ,0, 0],
                     [0         ,0          ,0, 0]], dtype='float')


def get_rotation(H):
    return H[:3,:3]

def get_position(H):
    return H[:3,3]

def calc_error(H1, H2):
    shape = np.array(H1).shape
    error = 0
    for i in range(shape[0]):
        for j in range(shape[1]):
            error += abs(H1[i,j] - H2[i,j])
    return error

def check_diff(eq1, eq2):
    return simplify(eq1 - eq2) == 0

def print_matrix(f):
        print(f"Homogeneous Matrix:\n{f}")
        print("Rotation:\n", get_rotation(f))
        print("Position:\n", get_position(f)) 

def pos2hom(pos):
    hom = np.zeros((4,4))
    hom[:3,3] = pos.T
    hom[3,3] = 1
    return hom