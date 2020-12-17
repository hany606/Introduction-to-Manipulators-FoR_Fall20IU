import numpy as np
from utils import *
import sympy as sp


class EulerLagrange2:
    def __init__(self):
        self.gravity = 9.81
        self.inertia = [1, 2]
        self.mass = [3, 4]
        self.lengths = [2,2]
        self.n = 2
        
        self.simp = sp.simplify
        self.mat = sp.Matrix
        
        self.q1, self.q2 = sp.symbols("q1 q2", real=True)
        self.q = [self.q1, self.q2]
        self.dq1, self.dq2 = sp.symbols("dq1 dq2", real=True)
        self.dq = [self.dq1, self.dq2]
        self.ddq1, self.ddq2 = sp.symbols("ddq1 ddq2", real=True)
        self.ddq = [self.ddq1, self.ddq2]
        self.l1, self.l2 = sp.symbols("l1 l2", real=True)
        self.l = [self.l1, self.l2]
        self.I1, self.I2 = sp.symbols("I1 I2", real=True)
        self.I = [self.I1, self.I2]
        self.m1, self.m2 = sp.symbols("m1 m2", real=True)
        self.m = [self.m1, self.m2]
        self.g = sp.symbols("g", real=True)
        self.g_vector = self.mat(([[0], [-1*self.g], [0]]))
        
        self.zero_vec = self.mat(np.array([0,0,0]).reshape(3,1))
        # Transition matrices
        self.A1 = sp_rotation_z(self.q1) * sp_translation_x(self.l1) # A1
        self.A2 = self.A1 * sp_rotation_z(self.q2) * sp_translation_x(self.l2) # A2
        self.A = [self.A1, self.A2]
        
        self.R1 = self.A1[:3,:3] 
        self.R2 = self.A2[:3,:3]
        self.R = [self.R1, self.R2]
        
        # Positions of the center of mass with respect to the zero world frame
        self.oc0 = self.zero_vec
        self.oc1 = self.zero_vec
        self.oc2 = ((self.A1 * sp_rotation_z(self.q2) * sp_translation_x(0.5*self.l2))[:3,3])
        self.oc = [self.oc1, self.oc2]
        
        # Positions of the joints o0 -> joint1, o1 -> joint2
        self.o0 = self.zero_vec
        self.o1 = self.A1[:3,3]
        self.o = [self.o0, self.o1]
        
        # Calculations of z vectors: According to the axis of the joint in the transition matrix
        self.z0 = self.mat(np.array([0,0,1]).reshape(3,1)) # from A0 which is I
        self.z1 = self.A1[:3,2] # from A1
        self.z = [self.z0, self.z1]
        
    def _calc_dynamics(self):
        def _coriolis(M, q, dq):
            C = self.mat(np.array([[0,0],[0,0]]).reshape((self.n,self.n)))
            for k in range(self.n):
                for j in range(self.n):
                    for i in range(self.n):
                        c_ijk = 0.5*(M[k, j].diff(q[i])+M[k, i].diff(q[j])+M[i,j].diff(q[k]))
                        C[k, j] = C[k, j]+c_ijk*dq[i]
            return C      

        self.Jv = []
        self.Jw = []
        M = self.mat(np.zeros((self.n, self.n)))
        P = 0
        G = self.mat()

        for i in range(self.n):
            self.Jv.append(self.mat())
            self.Jw.append(self.mat())
            for j in range(self.n):
                Jvij = None
                Jwij = None
                if(j <= i):
                    Jvij = self.z[j].cross(self.oc[i] - self.o[j])
                    Jwij = self.z[j]
                else:
                    Jvij = self.zero_vec
                    Jwij = self.zero_vec
                    
                self.Jv[i] = self.Jv[i].col_insert(j,Jvij)
                self.Jw[i] = self.Jw[i].col_insert(j,Jwij)
        
            Mi = self.m[i]*self.Jv[i].T * self.Jv[i] + self.Jw[i].T * self.R[i] * self.I[i] * self.R[i].T * self.Jw[i]            
            M += Mi

            Pi = self.m[i]*self.g_vector.T.dot(-1*self.oc[i])
            P += Pi

        for i in range(self.n):
            Gi = self.mat([P.diff(self.q[i])])
            G = G.row_insert(i, Gi)
            
        C = _coriolis(M, [self.q1, self.q2], [self.dq1, self.dq2])
        
        return M, C, G
    
    def direct(self, q0, dq0, ut, dt=0.0004, debug=False):
        M, C, G = self._calc_dynamics()
        qt = [q0]
        dqt = [dq0]
        ddqt = [np.array([0,0]).reshape(2,1)]
        
        for i in range(1,len(ut)+1):
            q = qt[i-1].copy().squeeze()
            dq = dqt[i-1].copy().squeeze()
            ddq = ddqt[i-1].copy().squeeze()
            u = ut[i-1].copy()
            if(debug):
                print(q[1])
            
            M_np = np.array(M.subs({self.q1:q[0], self.q2:q[1],
                           self.I1:self.inertia[0], self.I2:self.inertia[1],
                           self.m1:self.mass[0], self.m2:self.mass[1],
                           self.l1:self.lengths[0], self.l2:self.lengths[1]})).astype(np.float)
            
            C_np = np.array(C.subs({self.q1:q[0], self.q2:q[1],
                                    self.dq1:dq[0], self.dq2:dq[1],
                                    self.I1:self.inertia[0], self.I2:self.inertia[1],
                                    self.m1:self.mass[0], self.m2:self.mass[1],
                                    self.l1:self.lengths[0], self.l2:self.lengths[1]})).astype(np.float)
            
            G_np = np.array(G.subs({self.q1:q[0], self.q2:q[1],
                           self.I1:self.inertia[0], self.I2:self.inertia[1],
                           self.m1:self.mass[0], self.m2:self.mass[1],
                           self.l1:self.lengths[0], self.l2:self.lengths[1],
                           self.g:self.gravity})).astype(np.float)
                                    
            # print(M_np)
            # Semi-implicit Euler integration -> https://en.wikipedia.org/wiki/Semi-implicit_Euler_method#:~:text=The%20semi%2Dimplicit%20Euler%20is,integrator%2C%20unlike%20the%20standard%20method.&text=This%20is%20clear%20advantage%20over,standard)%20Euler%20and%20backward%20Euler.
            ddq = np.linalg.inv(M_np) @ (u-G_np-(C_np@(dq.reshape(2,1))))  
            dq = dq.reshape(2,1) + ddq*dt
            q = q.reshape(2,1) + dq*dt
            if(debug):
                print(q)
                print(dq)
                print(ddq)
                print("UUUU")
                print(u)
            if(debug):
                print("--------------")
            # ddqt.append([ddq[1], ddq[0]])
            # dqt.append([dq[1], dq[0]])
            # qt.append([q[1], q[0]])
            ddqt.append(ddq)
            dqt.append(dq)
            qt.append(q)
                        
        return qt, dqt, ddqt
    
    def inverse(self, qt, dqt, ddqt):
        M, C, G = self._calc_dynamics()
        ut = []
        for i in range(0,len(qt)):
            q = qt[i].copy().squeeze()
            dq = dqt[i].copy().squeeze()
            ddq = ddqt[i].copy().squeeze()
            
            M_np = np.array(M.subs({self.q1:q[0], self.q2:q[1],
                           self.I1:self.inertia[0], self.I2:self.inertia[1],
                           self.m1:self.mass[0], self.m2:self.mass[1],
                           self.l1:self.lengths[0], self.l2:self.lengths[1]}))
            
            C_np = np.array(C.subs({self.q1:q[0], self.q2:q[1],
                                    self.dq1:dq[0], self.dq2:dq[1],
                                    self.I1:self.inertia[0], self.I2:self.inertia[1],
                                    self.m1:self.mass[0], self.m2:self.mass[1],
                                    self.l1:self.lengths[0], self.l2:self.lengths[1]}))
            
            G_np = np.array(G.subs({self.q1:q[0], self.q2:q[1],
                           self.I1:self.inertia[0], self.I2:self.inertia[1],
                           self.m1:self.mass[0], self.m2:self.mass[1],
                           self.l1:self.lengths[0], self.l2:self.lengths[1],
                           self.g:self.gravity}))
                                    
            u = np.array(M_np@(ddq.reshape(2,1)) + C_np@(dq.reshape(2,1)) + G_np).reshape(2,1)
            ut.append(u)
            print(i)
            
        return np.array(ut).astype(np.float)
        
class EulerLagrange:
    def __init__(self):
        self.length = 0.4
        self.l = [2*self.length, 2*self.length]
        self.d = [0, self.length]
        self.gravity = 9.81
        self.inertia = [1, 2]
        self.mass = [3, 4]
        
    def direct(self, q0, dq0, ut, dt=0.0004, debug=False):
        a1 = self.inertia[0] + self.mass[0]*(self.d[0]**2) + self.inertia[1] + self.mass[1]*(self.d[1]**2) + self.mass[1]*(self.l[0]**2)
        a2 = self.mass[1]*self.l[0]*self.d[1]
        a3 = self.inertia[1] + self.mass[1]*(self.d[1]**2)
        a4 = self.gravity * (self.mass[0]*self.d[0] + self.mass[1]*self.l[0])
        a5 = self.gravity * (self.mass[1]*self.d[1])

        qt = [q0]
        dqt = [dq0]
        ddqt = [np.array([0,0]).reshape(2,1)]
        
        for i in range(1,len(ut)+1):
            q = qt[i-1].copy()
            dq = dqt[i-1].copy()
            ddq = ddqt[i-1].copy()
            u = ut[i-1].copy()
            if(debug):
                print(q[1])
            M = np.array([[a1+2*a2*np.cos(q[1]), a3+a2*np.cos(q[1])], [a3+a2*np.cos(q[1]), a3]]).astype(np.float64)
            
            c = np.array([-a2*np.sin(q[1]*(dq[1]**2+2*dq[0]*dq[1])), a2*np.sin(q[1])*(dq[1]**2)]).reshape(2,1)
            
            C = np.array([[-2*a2*np.sin(q[1])*dq[1], -a2*np.sin(q[1])*dq[1]], [a2*np.sin(q[1])*dq[0], 0]]).reshape(2,2)
            
            G = np.array([a4*np.cos(q[0]) + a5*np.cos(q[0]+q[1]), a5*np.cos(q[0]+q[1])]).reshape(2,1)
            # q = q + dq*dt
            # dq = dq + ddq*dt
            # ddq = np.linalg.inv(M) @ (u-G-(C@dq).astype(np.float64))    

            # Semi-implicit Euler integration -> https://en.wikipedia.org/wiki/Semi-implicit_Euler_method#:~:text=The%20semi%2Dimplicit%20Euler%20is,integrator%2C%20unlike%20the%20standard%20method.&text=This%20is%20clear%20advantage%20over,standard)%20Euler%20and%20backward%20Euler.
            # print("C", (C@dq).astype(np.float64))
            # print("c", c)
            ddq = np.linalg.inv(M) @ (u-G-(C@dq).astype(np.float64))   
            dq = dq + ddq*dt
            q = q + dq*dt
            if(debug):
                print(q)
                print(dq)
                print(ddq)
                print("UUUU")
                print(u)
            if(debug):
                print("--------------")
            # ddqt.append([ddq[1], ddq[0]])
            # dqt.append([dq[1], dq[0]])
            # qt.append([q[1], q[0]])
            ddqt.append(ddq)
            dqt.append(dq)
            qt.append(q)
            
        
        return qt, dqt, ddqt
    
    def inverse(self, qt, dqt, ddqt):
        a1 = self.inertia[0] + self.mass[0]*(self.d[0]**2) + self.inertia[1] + self.mass[1]*(self.d[1]**2) + self.mass[1]*(self.l[0]**2)
        a2 = self.mass[1]*self.l[0]*self.d[1]
        a3 = self.inertia[1] + self.mass[1]*(self.d[1]**2)
        a4 = self.gravity * (self.mass[0]*self.d[0] + self.mass[1]*self.l[0])
        a5 = self.gravity * (self.mass[1]*self.d[1])
        
        ut = []
        for i in range(0,len(qt)):
            q = qt[i].copy()
            dq = dqt[i].copy()
            ddq = ddqt[i].copy()
            
            M = np.array([[a1+2*a2*np.cos(q[1]), a3+a2*np.cos(q[1])], [a3+a2*np.cos(q[1]), a3]]).squeeze()
            
            c = np.array([-a2*np.sin(q[1]*(dq[1]**2+2*dq[0]*dq[1])), a2*np.sin(q[1])*(dq[1]**2)]).reshape(2,1)
            C = np.array([[-2*a2*np.sin(q[1])*dq[1], -a2*np.sin(q[1])*dq[1]], [a2*np.sin(q[1])*dq[0], 0]]).reshape(2,2)
            G = np.array([a4*np.cos(q[0]) + a5*np.cos(q[0]+q[1]), a5*np.cos(q[0]+q[1])]).reshape(2,1)
            
            ut.append(M@ddq + (C@dq).astype(np.float64) + G)
            
        return ut
    
    
if __name__ == "__main__":
    dyn = EulerLagrange2()
    M, G, C = dyn._calc_dynamics()
    from EulerLagrangeDynamics import EulerLagrange2
    M2, G2, C2 = dyn._calc_dynamics()
    print(C-C2)
    # print(G2)
    # dyn = EulerLagrange()
    # q0 = np.array([-np.pi/2, -np.pi/2]).reshape(2,1)
    # q0 = np.array([-np.pi/2, 0]).reshape(2,1)
    # dq0 = np.array([0,0]).reshape(2,1)
    # ut = [np.array([0,0]).reshape(2,1) for i in range(10)]
    # qt, dqt, ddqt = dyn.direct(q0, dq0, ut)
    # # print(dqt[-1])
    # ut2 = dyn.inverse(qt, dqt, ddqt)
    # print(ut2)
    