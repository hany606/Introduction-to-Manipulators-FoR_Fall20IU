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


A =  [mat(rotation_z(0)),
                    mat(rotation_z(q0)) * mat(translation_z(l0)) * mat(translation_x(-l1)),
                    mat(rotation_x(q1)) * mat(translation_z(l2)),
                    mat(translation_z(q2)),
                    mat(rotation_z(q3)),
                    mat(rotation_x(q4)),
                    mat(rotation_x(q5)) * mat(translation_z(l3))]
# calculate O, U vectors
O = []
U = []
# z y y x y x
u_rotation_joints_cols = [2, 0, 2, 2, 0, 2]
T0i = np.eye(4)
for i in range(6):
    T0i = T0i * A[i]
    O.append(T0i[:3,3])
    U.append(T0i[:3, u_rotation_joints_cols[i]])
T0i = T0i * A[6]
O.append(T0i[:3,3])
print(O, U)
J = np.zeros((6,6))
for i in range(6):
    if(i == 2):
        J[:3,i] = U[i]
        J[3:,i] = np.zeros((3,))
        continue
    
    J[:3,i] = ((U[i]).T ^ (O[6] - O[i]))
    J[3:,i] = U[i]
print(J)
# print(sp.Matrix(rotation_x(l1))*sp.Matrix(translation_y(l2)))

# Kinematics is in zero configuration
T_123 = mat(rotation_z(q0)) * mat(rotation_x(q1)) * mat(translation_x(-l1))*\
        mat(translation_z(l2)) * mat(translation_z(q2))


T_456 = mat(rotation_z(q3)) * mat(rotation_x(q4)) * mat(rotation_z(q5))

# print(T_123[0,1])

# for i in range(4):
#     for j in range(4):
#         print(T_123[i,j],end=",\t")
#     print("")
# print("\n\n\n\n-------------\n\n\n\n")
# print(T_456)

print("T_123")
print_mat(T_123)
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

print("T_456")
print_mat(T_456)
print("1st row of T_456")
print(T_456[0,:3])
# print("\n\n\n\n-------------\n\n\n\n")
print("2nd row of T_456")
print(T_456[1,:3])
# print("\n\n\n\n-------------\n\n\n\n")
print("3rd row of T_456")
print(T_456[2,:3])


# print("\n\n\n\n-------------\n\n\n\n")
print(sp.printing.latex(T_456))

# q1, q2, q3, q4, q5, q6 = 0,0,0,0,0,0
# l1, l2, l3, l4, l5, l6 = 0,0,0,0,0,0

# For position vector: in order to get q1, q2, q3
# x = l2*cos(q1) + l3*cos(q1)*cos(q2) + l4*(-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3)) + l5*(-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))
# y = l2*sin(q1) + l3*sin(q1)*cos(q2) + l4*(-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3)) + l5*(-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))
# z = -l3*sin(q2) + l4*(-sin(q2)*cos(q3) - sin(q3)*cos(q2)) + l5*(-sin(q2)*cos(q3) - sin(q3)*cos(q2))

# Full matrix to substitute in it q1, q2, q3 after getting them using the last column
# In order to be able to get its inverse and get T_{456}_{bits} = T_{123}^{-1} T_o = T_{456}
# T_123 = [
#             [-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3),	-sin(q1),	sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2),	l2*cos(q1) + l3*cos(q1)*cos(q2) + l4*(-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3)) + l5*(-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3)),]
#             [-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3),	cos(q1),	sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2),	l2*sin(q1) + l3*sin(q1)*cos(q2) + l4*(-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3)) + l5*(-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3)),]
#             [-sin(q2)*cos(q3) - sin(q3)*cos(q2),	0,	-sin(q2)*sin(q3) + cos(q2)*cos(q3),	-l3*sin(q2) + l4*(-sin(q2)*cos(q3) - sin(q3)*cos(q2)) + l5*(-sin(q2)*cos(q3) - sin(q3)*cos(q2)),]
#             [0,	0,	0,	1,]
#         ]

# For rotation matrix: in order to get q4, q5, q6
# cos(q5),	sin(q5)*sin(q6),	sin(q5)*cos(q6)
# sin(q4)*sin(q5),	-sin(q4)*sin(q6)*cos(q5) + cos(q4)*cos(q6),	-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4)	
# -sin(q5)*cos(q4),	sin(q4)*cos(q6) + sin(q6)*cos(q4)*cos(q5),	-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6)