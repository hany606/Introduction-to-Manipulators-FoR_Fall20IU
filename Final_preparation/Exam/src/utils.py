import numpy as np
import sympy as sp
from matplotlib import pyplot as plt

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


def rotation_x3(theta):
    return np.array([[1,         0,          0],
                     [0,np.cos(theta),-np.sin(theta)],
                     [0,np.sin(theta), np.cos(theta)]], dtype='float')


def rotation_y3(theta):
    return np.array([[np.cos(theta) ,0,np.sin(theta)],
                     [0          ,1,         0],
                     [-np.sin(theta),0,np.cos(theta)]], dtype='float')

def rotation_z3(theta):
    return np.array([[np.cos(theta),-np.sin(theta),0],
                     [np.sin(theta), np.cos(theta),0],
                     [0         ,0          ,1]], dtype='float')



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


def sp_translation_x(l):
    return sp.Matrix(np.array([[1,0,0, l],
                     [0,1,0, 0],
                     [0,0,1, 0],
                     [0,0,0, 1]]))

def sp_translation_y(l):
    return sp.Matrix(np.array([[1,0,0, 0],
                     [0,1,0, l],
                     [0,0,1, 0],
                     [0,0,0, 1]]))

def sp_translation_z(l):
    return sp.Matrix(np.array([[1,0,0, 0],
                     [0,1,0, 0],
                     [0,0,1, l],
                     [0,0,0, 1]]))

def sp_rotation_x(theta):
    return sp.Matrix(np.array([[1,         0,          0, 0],
                     [0,sp.cos(theta),-sp.sin(theta), 0],
                     [0,sp.sin(theta), sp.cos(theta), 0],
                     [0,         0,          0, 1]]))


def sp_rotation_y(theta):
    return sp.Matrix(np.array([[sp.cos(theta) ,0,sp.sin(theta), 0],
                     [0          ,1,         0, 0],
                     [-sp.sin(theta),0,sp.cos(theta), 0],
                     [0          ,0,         0, 1]]))

def sp_rotation_z(theta):
    return sp.Matrix(np.array([[sp.cos(theta),-sp.sin(theta),0, 0],
                     [sp.sin(theta), sp.cos(theta),0, 0],
                     [0         ,0          ,1, 0],
                     [0         ,0          ,0, 1]]))

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
    return sp.simplify(eq1 - eq2) == 0

def print_matrix(f):
        print(f"Homogeneous Matrix:\n{f}")
        print("Rotation:\n", get_rotation(f))
        print("Position:\n", get_position(f)) 

def pos2hom(pos):
    hom = np.zeros((4,4))
    hom[:3,3] = pos.T
    hom[3,3] = 1
    return hom

def plot_u(u, dt=1/100, title="Control Input", time=None):
    
    u = np.array(u).reshape(len(u), 2)
    time = np.linspace(0, dt*len(u), len(u)) if time is None else time

    fig, ax = plt.subplots(1)
    ax.plot(time, u)
    ax.set_xlabel("Time - seconds")
    ax.set_ylabel("u - (torque -- N.m)")
    ax.legend(["Joint1", "Joint2"], loc="upper left", bbox_to_anchor=(1, 1))
    ax.set_title("Control - Torques on Joints")
    
    fig.suptitle(title, fontsize=12)
    plt.tight_layout()
    plt.show()

def plot_trajectory(traj, dt=1/100, title="Trajectory", time=None):
    q, dq, ddq = traj[:]
    
    q = np.array(q).squeeze()
    dq = np.array(dq).squeeze()
    ddq = np.array(ddq).squeeze()
    
    time = np.linspace(0, dt*len(q), len(q)) if time is None else time

    fig, axs = plt.subplots(3,1)
    axs[0].plot(time, q)
    axs[0].set_xlabel("Time - seconds")
    axs[0].set_ylabel("q - rad")
    axs[0].legend(["Joint1", "Joint2"], loc="upper left", bbox_to_anchor=(1, 1))
    axs[0].set_title("Position")

    axs[1].plot(time, dq)
    axs[1].set_xlabel("Time - seconds")
    axs[1].set_ylabel("dq - rad/sec")
    axs[1].legend(["Joint1", "Joint2"], loc="upper left", bbox_to_anchor=(1, 1))
    axs[1].set_title("Velocity")
    
    axs[2].plot(time, ddq)
    axs[2].set_xlabel("Time - seconds")
    axs[2].set_ylabel("dq - rad/sec^2")
    axs[2].legend(["Joint1", "Joint2"], loc="upper left", bbox_to_anchor=(1, 1))
    axs[2].set_title("Acceleration")

    
    fig.suptitle(title, fontsize=12)
    plt.tight_layout()
    plt.show()
    
