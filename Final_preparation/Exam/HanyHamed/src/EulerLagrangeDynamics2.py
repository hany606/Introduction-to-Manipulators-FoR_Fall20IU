import numpy as np
from utils import *
import sympy as sp
from tqdm import tqdm


class EulerLagrange2:
    def __init__(self):
        self.gravity = 9.81
        self.inertia = [10, 4, 1]
        self.mass = [10, 5, 1]
        self.lengths = [1, 0.5, 0.2]
        self.height_offset = 10
        self.n = 3
        self.joint_type = ["R", "P", "P"]
        
        self.simp = sp.simplify
        self.mat = sp.Matrix
        
        self.q1, self.q2, self.q3 = sp.symbols("q1 q2 q3", real=True)
        self.q = [self.q1, self.q2, self.q3]
        self.dq1, self.dq2, self.dq3 = sp.symbols("dq1 dq2 dq3", real=True)
        self.dq = [self.dq1, self.dq2, self.dq3]
        self.ddq1, self.ddq2, self.ddq3 = sp.symbols("ddq1 ddq2 ddq3", real=True)
        self.ddq = [self.ddq1, self.ddq2, self.ddq3]
        self.l1, self.l2, self.l3 = sp.symbols("l1 l2 l3", real=True)
        self.l = [self.l1, self.l2, self.l3]
        self.I1, self.I2, self.I3 = sp.symbols("I1 I2 I3", real=True)
        self.I = [self.I1, self.I2, self.I3]
        self.m1, self.m2, self.m3 = sp.symbols("m1 m2 m3", real=True)
        self.m = [self.m1, self.m2, self.m3]
        self.g = sp.symbols("g", real=True)
        self.g_vector = self.mat(([[0], [-1*self.g], [0]]))
        self.h = sp.symbols("h", real=True)
        
        self.zero_vec = self.mat(np.array([0,0,0]).reshape(3,1))
        # Transition matrices
        self.A1 = sp_rotation_x(self.q1) * sp_translation_x(self.l1) # A1
        self.A2 = self.A1 * sp_translation_x(self.q2) * sp_translation_x(self.l2) # A2
        self.A3 = self.A2 * sp_translation_z(self.q3) * sp_translation_z(-self.l3) # A3
        self.A = [self.A1, self.A2, self.A3]
        
        self.R1 = self.A1[:3,:3] 
        self.R2 = self.A2[:3,:3]
        self.R3 = self.A3[:3,:3]
        self.R = [self.R1, self.R2, self.R3]
        
        # Positions of the center of mass with respect to the zero world frame
        self.oc0 = self.mat(np.array([self.h, 0, 0]).reshape((3,1)))
        self.oc1 = self.mat(np.array([self.h+0.5*self.l1, 0, 0]).reshape((3,1)))
        self.oc2 = ((self.A1 * sp_translation_x(self.q2) * sp_translation_x(0.5*self.l2))[:3,3])
        self.oc3 = ((self.A2 * sp_translation_z(self.q3) * sp_translation_z(0.5*self.l3))[:3,3])
        self.oc = [self.oc1, self.oc2, self.oc3]
        
        # Positions of the joints o0 -> joint1, o1 -> joint2
        self.o0 = self.mat(np.array([self.h, 0, 0]).reshape((3,1)))
        self.o1 = self.A1[:3,3]
        self.o2 = self.A2[:3,3]
        self.o = [self.o0, self.o1, self.o2]
        
        # Calculations of z vectors: According to the axis of the joint in the transition matrix
        self.z0 = self.mat(np.array([0,1,0]).reshape(3,1)) # from A0 which is I
        self.z1 = self.A1[:3,0] # from A1
        self.z2 = self.A2[:3,2]
        self.z = [self.z0, self.z1, self.z2]
        
    def _calc_dynamics(self):
        def _coriolis(M, q, dq):
            C = self.mat(np.zeros((self.n,self.n)))
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
                    if(self.joint_type[i] == "R"):
                        Jvij = self.z[j].cross(self.oc[i] - self.o[j])
                        Jwij = self.z[j]
                    else:
                        Jvij = self.z[j]
                        Jwij = self.zero_vec
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
            
        C = _coriolis(M, self.q, self.dq)
        
        return M, C, G
    
    def direct(self, q0, dq0, ut, dt=0.0004, debug=False):
        M, C, G = self._calc_dynamics()
        qt = [q0.reshape(3,1)]
        dqt = [dq0.reshape(3,1)]
        ddqt = [np.zeros((self.n, 1))]
        
        for i in tqdm(range(1,len(ut)+1)):
            q = qt[i-1].copy().squeeze()
            dq = dqt[i-1].copy().squeeze()
            ddq = ddqt[i-1].copy().squeeze()
            u = ut[i-1].copy()
            if(debug):
                print(q[1])
            
            M_np = np.array(M.subs({self.q1:q[0], self.q2:q[1], self.q3:q[2],
                                    self.I1:self.inertia[0], self.I2:self.inertia[1], self.I3:self.inertia[2],
                                    self.m1:self.mass[0], self.m2:self.mass[1], self.m3:self.mass[2],
                                    self.l1:self.lengths[0], self.l2:self.lengths[1], self.l3:self.lengths[2],
                                    self.h:self.height_offset})).astype(np.float)
            # print(M_np)
            # M_np = M_np.astype(np.float)
            
            C_np = np.array(C.subs({self.q1:q[0], self.q2:q[1], self.q3:q[2],
                                    self.dq1:dq[0], self.dq2:dq[1], self.dq3:dq[2],
                                    self.I1:self.inertia[0], self.I2:self.inertia[1], self.I3:self.inertia[2],
                                    self.m1:self.mass[0], self.m2:self.mass[1], self.m3:self.mass[2],
                                    self.l1:self.lengths[0], self.l2:self.lengths[1], self.l3:self.lengths[2],
                                    self.h:self.height_offset})).astype(np.float64)
            
            G_np = np.array(G.subs({self.q1:q[0], self.q2:q[1], self.q3:q[2],
                                    self.I1:self.inertia[0], self.I2:self.inertia[1], self.I3:self.inertia[2],
                                    self.m1:self.mass[0], self.m2:self.mass[1], self.m3:self.mass[2],
                                    self.l1:self.lengths[0], self.l2:self.lengths[1], self.l3:self.lengths[2],
                                    self.g:self.gravity, self.h:self.height_offset})).astype(np.float64)
                       
            # Semi-implicit Euler integration -> https://en.wikipedia.org/wiki/Semi-implicit_Euler_method#:~:text=The%20semi%2Dimplicit%20Euler%20is,integrator%2C%20unlike%20the%20standard%20method.&text=This%20is%20clear%20advantage%20over,standard)%20Euler%20and%20backward%20Euler.
            ddq = np.linalg.inv(M_np) @ (u.reshape(self.n,1)-G_np-(C_np@(dq.reshape(self.n,1)))) 
            dq = dq.reshape(self.n,1) + ddq*dt
            q = q.reshape(self.n,1) + dq*dt
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
        for i in tqdm(range(0,len(qt))):
            q = qt[i].copy().squeeze()
            dq = dqt[i].copy().squeeze()
            ddq = ddqt[i].copy().squeeze()
            
            M_np = np.array(M.subs({self.q1:q[0], self.q2:q[1], self.q3:q[2],
                                    self.I1:self.inertia[0], self.I2:self.inertia[1], self.I3:self.inertia[2],
                                    self.m1:self.mass[0], self.m2:self.mass[1], self.m3:self.mass[2],
                                    self.l1:self.lengths[0], self.l2:self.lengths[1], self.l3:self.lengths[2],
                                    self.h:self.height_offset}))
            
            C_np = np.array(C.subs({self.q1:q[0], self.q2:q[1], self.q3:q[2],
                                    self.dq1:dq[0], self.dq2:dq[1], self.dq3:dq[2],
                                    self.I1:self.inertia[0], self.I2:self.inertia[1], self.I3:self.inertia[2],
                                    self.m1:self.mass[0], self.m2:self.mass[1], self.m3:self.mass[2],
                                    self.l1:self.lengths[0], self.l2:self.lengths[1], self.l3:self.lengths[2],
                                    self.h:self.height_offset}))
            
            G_np = np.array(G.subs({self.q1:q[0], self.q2:q[1], self.q3:q[2],
                                    self.I1:self.inertia[0], self.I2:self.inertia[1], self.I3:self.inertia[2],
                                    self.m1:self.mass[0], self.m2:self.mass[1], self.m3:self.mass[2],
                                    self.l1:self.lengths[0], self.l2:self.lengths[1], self.l3:self.lengths[2],
                                    self.g:self.gravity, self.h:self.height_offset}))
                                    
            u = np.array(M_np@(ddq.reshape(self.n,1)) + C_np@(dq.reshape(self.n,1)) + G_np).reshape(self.n,1)
            ut.append(u)
            # print(i)
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
    from TrajectoryPlanning import TrajectoryPlanning
    dyn = EulerLagrange2()
    M, G, C = dyn._calc_dynamics()
    print("--------------------- Polynomial 3rd ordedr ---------------------")
    (q0, qf) = ([0, 0, 0], [0.2, 1, 0.5])
    t0,tf = 0, 2
    (dq0, dqf) = ([0,0,0], [0,0,0])
    traj_poly3 = TrajectoryPlanning.polynomial3(t0, q0, dq0, tf, qf, dqf)
    # TrajectoryPlanning.plot_trajectory(traj=traj_poly3, title="Polynomial - 3rd Order")

    qt, dqt, ddqt = TrajectoryPlanning.extract_traj(traj_poly3)
    ut = dyn.inverse(qt, dqt, ddqt)
    plot_u(ut, n=3)
    
    # print("--------------------- Polynomial 1st ordedr ---------------------")
    # (u0, uf) = ([0, 0, -10], [4, 0, -6])
    # t0,tf = 0, 2
    # traj_poly1 = TrajectoryPlanning.polynomial1_tor(t0, u0, tf, uf)
    # ut = traj_poly1.squeeze()[:]
    # # TrajectoryPlanning.plot_trajectory(traj=traj_poly3, title="Polynomial - 3rd Order")
    # # plot_u(ut, n=3)
    # (q0, dq0) = (np.array([0, 0, 0]), np.array([0, 0, 0]))
    # dt= 1/1000
    # qt, dqt, ddqt = dyn.direct(q0, dq0, ut, dt=1/1000)
    # plot_trajectory([qt, dqt, ddqt],n=3, dt=dt)

     