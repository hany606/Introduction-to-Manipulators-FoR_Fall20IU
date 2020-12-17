import numpy as np
from utils import *

class NewtonEuler:
    def __init__(self,):
        self.length = 0.4
        self.l = [2*self.length, 2*self.length]
        self.d = [0, self.length]
        self.a = [self.l[0]-self.d[0], self.l[1]-self.d[1]]
        self.gravity = 9.81
        self.inertia = [1, 2]
        self.mass = [3, 4]
        
    def inverse(self, qt, dqt, ddqt):
        ut = []
        for j in range(len(ddqt)):
            w_i = [np.array([0, 0, 0]).reshape(3,1)]
            dw_i = [np.array([0, 0, 0]).reshape(3,1)]
            a_i = [np.array([0, +self.gravity, 0]).reshape(3,1)]    # \vec a_0 - \vec g
            ac_i = [np.array([0, 0, 0]).reshape(3,1)]
            f_i = [None, None, None, np.array([0, 0, 0]).reshape(3,1)]
            tau_i = [None, None, None, np.array([0, 0, 0]).reshape(3,1)]
            u = []
            
            q = qt[j].copy()
            dq = dqt[j].copy()
            ddq = ddqt[j].copy()
            
            R_im1_i = [None, rotation_z3(q[0]),
                             rotation_z3(q[1]),
                             np.eye(3)]
            
            r_i_ci = [None, np.array([self.d[0]*np.cos(q[0]), 0, 0]).reshape(3,1).astype(np.float64),  # r_1,c1
                            np.array([self.d[1]*np.cos(q[1]), 0, 0]).reshape(3,1).astype(np.float64)]  # r_2,c2
            r_im1_i = [None, np.array([self.a[0], 0, 0]).reshape(3,1),  # r_0,1
                             np.array([self.a[1], 0, 0]).reshape(3,1)]  # r_1,2
            z_i = [None, np.array([0, 0, 1]).reshape(3,1), np.array([0, 0, 1]).reshape(3,1)]
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
            for i in range(2):
                # u.append(f_i[i+1].T*z_i[i+1])
                u.append(tau_i[i+1].T@z_i[i+1])
            
            ut.append(u)
        return ut


if __name__ == "__main__":
    from EulerLagrangeDynamics import EulerLagrange
    EL_dyn = EulerLagrange()
    NE_dyn = NewtonEuler()
    
    # q0 = np.array([-np.pi/2, np.pi/2]).reshape(2,1)
    q0 = np.array([-np.pi/2, 0]).reshape(2,1)
    x = 0.8*np.cos(q0[0]) + 0.8*np.cos(q0[0]+q0[1])
    y = 0.8*np.sin(q0[0]) + 0.8*np.sin(q0[0]+q0[1])
    print(x,y)
    # q0 = np.array([0, 0]).reshape(2,1)
    # print(q0)
    dq0 = np.array([0,0]).reshape(2,1)
    ut = [np.array([0,0]).reshape(2,1) for i in range(1000)]
    qt, dqt, ddqt = EL_dyn.direct(q0, dq0, ut)
    # print(min(np.array(qt)[:,1]))
    # print(max(np.array(qt)[:,1]))
    # print(sum(np.array(qt)[:,1])/len(qt))

    # print(min(np.array(qt)[:,0]))
    # print(max(np.array(qt)[:,0]))
    # print(sum(np.array(qt)[:,0])/len(qt))
    print(qt[-1])
    print(dqt[-1])
    print(ddqt[-1])
    
    # qt = [np.array([-np.pi/2, -np.pi/2]).reshape(2,1) for i in range(1000)]
    # dqt = [np.array([0, 0,]).reshape(2,1) for i in range(1000)]
    # ddqt = [np.array([0.05,0]).reshape(2,1) for i in range(1000)]


    # ut_ne_inverse = NE_dyn.inverse(qt, dqt, ddqt)
    # ut_el_inverse = EL_dyn.inverse(qt, dqt, ddqt)

    # print(ut[0])
    # # print(ut_ne_inverse[:5])
    # # print(ut_el_inverse[:5])
    # print(sum(np.array(ut_ne_inverse).squeeze()-np.array(ut_el_inverse).squeeze()))

    # # print(ut_ne_inverse[-1])
    
    # # print(sum(np.array(ut).squeeze()-np.array(ut_ne_inverse).squeeze()))
    # # print(sum(np.array(ut).squeeze()-np.array(ut_el_inverse).squeeze()))
