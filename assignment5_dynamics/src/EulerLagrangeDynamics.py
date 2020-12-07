import numpy as np
from utils import *


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
    dyn = EulerLagrange()
    # q0 = np.array([-np.pi/2, -np.pi/2]).reshape(2,1)
    q0 = np.array([-np.pi/2, 0]).reshape(2,1)
    dq0 = np.array([0,0]).reshape(2,1)
    ut = [np.array([0,0]).reshape(2,1) for i in range(1000)]
    qt, dqt, ddqt = dyn.direct(q0, dq0, ut)
    # print(dqt[-1])
    ut2 = dyn.inverse(qt, dqt, ddqt)
    # print(ut[0], ut2[)
    # print(sum(ut2))
    # print(ut2[-1])
    print(sum(np.array(ut2[1:])-ut))
    # Visualize
    # Plot data
    