import numpy as np
from utils import *
from tqdm import tqdm

class NewtonEuler:
    def __init__(self,):
        self.l = [1, 0.5, 0.2]
        self.d = [self.l[0]/2, self.l[1]/2, self.l[2]/2]
        self.a = [self.l[0]-self.d[0], self.l[1]-self.d[1], self.l[2]-self.d[2]]
        self.gravity = 9.81
        self.n = 3
       
        self.inertia = [10, 4, 1]
        self.mass = [10, 5, 1]
        self.height_offset = 10
        self.n = 3
        self.joint_type = ["R", "P", "P"]
        # self.joint_axis = [0, 1, 2]

    def direct(self, q0, dq0, ut, dt=0.0004):
        qt = [q0.reshape(3,1)]
        dqt = [dq0.reshape(3,1)]
        ddqt = [np.zeros((self.n, 1))]
        zeros = np.array([np.zeros((self.n, 1))])
        for i in tqdm(range(1,len(ut)+1)):
            q = qt[i-1].copy().squeeze()
            dq = dqt[i-1].copy().squeeze()
            ddq = ddqt[i-1].copy().squeeze()
            u = ut[i-1].copy()
            
            n = np.array(self.inverse2(np.array([q]), np.array([dq]), np.array([zeros]))).squeeze().reshape((3,1))
            
            M = np.zeros((self.n, self.n))
            for j in range(self.n):
                e_i = np.eye(self.n)[j] # np.array([0,0,1]).reshape((3,1))
                Mi = np.array(self.inverse2(np.array([q]), np.array([zeros]), np.array([e_i]))).squeeze().reshape((3,))
                M[:,j] = Mi
            # print(np.linalg.pinv(M))
            # print(u, n)
            ddq = np.linalg.pinv(M) @ (u.reshape((3,1))-n)
            dq = dq.reshape(self.n,1) + ddq*dt
            q = q.reshape(self.n,1) + dq*dt
            
            ddqt.append(ddq)
            dqt.append(dq)
            qt.append(q)
        return qt, dqt, ddqt


    def inverse2(self, qt, dqt, ddqt):
        ut = []
        for j in range(len(ddqt)):
            w_i = [np.array([0, 0, 0]).reshape(3,1)]
            dw_i = [np.array([0, 0, 0]).reshape(3,1)]
            a_i = [np.array([0, 0, +self.gravity]).reshape(3,1)]    # \vec a_0 - \vec g
            ac_i = [np.array([0, 0, 0]).reshape(3,1)]
            f_i = [None, None, None, None, np.array([0, 0, 0]).reshape(3,1)]
            tau_i = [None, None, None, None, np.array([0, 0, 0]).reshape(3,1)]
            u = []
            
            q = qt[j].copy().reshape(self.n,1)
            dq = dqt[j].copy().reshape(self.n,1)
            ddq = ddqt[j].copy().reshape(self.n,1)
            
            R_im1_i = [None, rotation_x3(q[0]),
                             np.eye(3),
                             np.eye(3),
                             np.eye(3)]
            
            r_i_ci = [None, np.array([self.d[0], 0, 0]).reshape(3,1).astype(np.float64),  # r_1,c1
                            np.array([self.d[1]+q[1], 0, 0]).reshape(3,1).astype(np.float64), # r_2,c2
                            np.array([0, 0, self.d[2]+q[2]]).reshape(3,1).astype(np.float64)] # r_3,c3
                            
            r_im1_i = [None, np.array([self.l[0]+q[1], 0, 0]).reshape(3,1).astype(np.float64),  # r_0,1
                             np.array([self.l[1]+q[2], 0, 0]).reshape(3,1).astype(np.float64),  # r_1,2
                             np.array([self.l[2], 0, 0]).reshape(3,1).astype(np.float64),  # r_2,3
                             ]  
            z_i = [None, np.array([1, 0, 0]).reshape(3,1), np.array([1, 0, 0]).reshape(3,1), np.array([0, 0, 1]).reshape(3,1)]
            # AR (Accelerations recursion)
            # input: q, dq, ddq
            # output: w, dw, a, ac
            for i in range(1,len(self.l)+1):
                w_i.append(R_im1_i[i].T @ (w_i[i-1]+ dq[i-1] * z_i[i]))
                # print(w_i[i])
                dw_i.append(R_im1_i[i].T @ (dw_i[i-1] \
                            + ddq[i-1] * z_i[i] + np.cross((dq[i-1]*w_i[i-1]).squeeze(), z_i[i].squeeze()).reshape(3,1)))
                # print(dw_i[i])
                a_i.append(   R_im1_i[i].T @ a_i[i-1] \
                            + np.cross(dw_i[i].squeeze(), r_im1_i[i].squeeze()).reshape(3,1) \
                            + np.cross(w_i[i].squeeze(),np.cross(w_i[i].squeeze(), r_im1_i[i].squeeze())).reshape(3,1))
                # print(a_i[i])
                ac_i.append(  a_i[i] \
                            + np.cross(dw_i[i].squeeze(), r_i_ci[i].squeeze()).reshape(3,1) \
                            + np.cross(w_i[i].squeeze(), np.cross(w_i[i].squeeze(), r_i_ci[i].squeeze())).reshape(3,1))
                # print(ac_i[i])
            # print("---------------------------")
            # F/TR (Forces/Torques recursion)
            # input: w, dw, a, ac
            # output: u
            for i in range(len(self.l),0, -1):
                f_i[i] = (R_im1_i[i+1]@f_i[i+1] + self.mass[i-1] * (ac_i[i]))
                # print(f_i[i])
                # In general, the inertia should be matrix but here it will not be different result as the matrix is zeros except (3,3) element with I_zz and the other vector that is multiplied with is [0,0, *]
                tau_i[i] = R_im1_i[i+1]@tau_i[i+1] \
                         - np.cross(f_i[i].squeeze(),(r_im1_i[i]+r_i_ci[i]).squeeze()).reshape(3,1) \
                         + (np.cross((R_im1_i[i+1]@f_i[i+1]).squeeze(), (r_i_ci[i]).squeeze()).reshape(3,1)) \
                         + self.inertia[i-1]*dw_i[i]+ np.cross(w_i[i].squeeze(),(self.inertia[i-1]*w_i[i]).squeeze()).reshape(3,1)
                # tau_i[i] = (R_im1_i[i+1]@tau_i[i+1] - np.cross(f_i[i].squeeze(),(r_i_ci[i]).squeeze()).reshape(3,1) + (np.cross((R_im1_i[i+1]@f_i[i+1]).squeeze(), (r_i_ci[i]-r_im1_i[i]).squeeze()).reshape(3,1)) + self.inertia[i-1]*dw_i[i]+ np.cross(w_i[i].squeeze(),(self.inertia[i-1]*w_i[i]).squeeze()).reshape(3,1))
                # tau_i[i] = (R_im1_i[i+1]@tau_i[i+1] - np.cross(f_i[i].squeeze(),(r_im1_i[i]+r_i_ci[i]).squeeze()).reshape(3,1) + R_im1_i[i+1]@(np.cross(f_i[i+1].squeeze(), r_i_ci[i].squeeze()).reshape(3,1)) + self.inertia[i-1]*dw_i[i]+ np.cross(w_i[i].squeeze(),(self.inertia[i-1]*w_i[i]).squeeze()).reshape(3,1))
                # print(tau_i[i])
            # FP
            for i in range(self.n):
                # u.append(f_i[i+1].T*z_i[i+1])
                if(self.joint_type[i] == "R"):
                    u.append(tau_i[i+1].T@z_i[i+1])
                else:
                    u.append(f_i[i+1].T@z_i[i+1])        
            ut.append(u)
        return ut


if __name__ == "__main__":
    dyn = NewtonEuler()
    from TrajectoryPlanning import TrajectoryPlanning
    print("--------------------- Polynomial 1st ordedr ---------------------")
    (u0, uf) = ([0, 0, -10], [4, 0, -6])
    t0,tf = 0, 2
    traj_poly1 = TrajectoryPlanning.polynomial1_tor(t0, u0, tf, uf)
    ut = traj_poly1.squeeze()[:]
    # TrajectoryPlanning.plot_trajectory(traj=traj_poly3, title="Polynomial - 3rd Order")
    # plot_u(ut, n=3)
    (q0, dq0) = (np.array([0, 0, 0]), np.array([0, 0, 0]))
    dt= 1/1000
    qt, dqt, ddqt = dyn.direct(q0, dq0, ut, dt=1/1000)
    plot_trajectory([qt, dqt, ddqt],n=3, dt=dt)
