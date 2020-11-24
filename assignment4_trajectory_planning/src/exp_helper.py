import sympy as sp
# from math import sin, cos
import numpy as np
import utils as ut

def print_mat(mat):
    for i in range(4):
        for j in range(4):
            print(mat[i,j],end=",\t")
        print("\n")
    print("\n\n\n\n-------------\n\n\n\n")

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
                     [0,sp.cos(theta),-sp.sin(theta), 0],
                     [0,sp.sin(theta), sp.cos(theta), 0],
                     [0,         0,          0, 1]])


def rotation_y(theta):
    return np.array([[sp.cos(theta) ,0,sp.sin(theta), 0],
                     [0          ,1,         0, 0],
                     [-sp.sin(theta),0,sp.cos(theta), 0],
                     [0          ,0,         0, 1]])

def rotation_z(theta):
    return np.array([[sp.cos(theta),-sp.sin(theta),0, 0],
                     [sp.sin(theta), sp.cos(theta),0, 0],
                     [0         ,0          ,1, 0],
                     [0         ,0          ,0, 1]])

mat = sp.Matrix
q0, q1, q2, q3, q4, q5 = sp.symbols("q0 q1 q2 q3 q4 q5", real=True)
l0, l1, l2, l3, l4, l5 = sp.symbols("l0 l1 l2 l3 l4 l5", real=True)

# print(sp.Matrix(rotation_x(l1))*sp.Matrix(translation_y(l2)))

# Kinematics is in zero configuration
T_123 = mat(rotation_z(q0)) * mat(rotation_y(q1)) *\
        mat(translation_x(l1)) * mat(rotation_y(q2)) * mat(translation_x(l2))

# print(T_123[0,1])

# for i in range(4):
#     for j in range(4):
#         print(T_123[i,j],end=",\t")
#     print("")
# print("\n\n\n\n-------------\n\n\n\n")

print("T_123")
# print_mat(T_123)
# print(T_123[:,3])
ut.print_matrix(T_123)
print("\n\n\n\n-------------\n\n\n\n")
print("1st element of T_123 position vector")
print(T_123[0,3])
# print("\n\n\n\n-------------\n\n\n\n")
print("2nd element of T_123 position vector")
print(T_123[1,3])
# print("\n\n\n\n-------------\n\n\n\n")
print("3rd element of T_123 position vector")
print(T_123[2,3])
print("\n\n\n\n-------------\n\n\n\n")


