import matplotlib.pyplot as plt
import numpy as np

class TrajectoryPlaning:

    # Take the trajectory as a parameter
    # Plot 3 
    @staticmethod
    def plot_trajectory(traj, dt=1/100, title="Trajectory"):
        traj = traj.squeeze()
        q, dq, ddq = traj[:,:, 0], traj[:,:, 1], traj[:,:, 2]
        time = np.linspace(0, dt*len(traj), len(traj))

        fig, axs = plt.subplots(3,1)
        axs[0].plot(time, q)
        axs[0].set_xlabel("Time - seconds")
        axs[0].set_ylabel("q - rad")
        axs[0].legend(["Joint1", "Joint2", "Joint3"], loc="upper left", bbox_to_anchor=(1, 1))
        axs[0].set_title("Position")

        axs[1].plot(time, dq)
        axs[1].set_xlabel("Time - seconds")
        axs[1].set_ylabel("dq - rad/sec")
        axs[1].legend(["Joint1", "Joint2", "Joint3"], loc="upper left", bbox_to_anchor=(1, 1))
        axs[1].set_title("Velocity")
        

        axs[2].plot(time, ddq)
        axs[2].set_xlabel("Time - seconds")
        axs[2].set_ylabel("dq - rad/sec^2")
        axs[2].legend(["Joint1", "Joint2", "Joint3"], loc="upper left", bbox_to_anchor=(1, 1))
        axs[2].set_title("Acceleration")

        
        fig.suptitle(title, fontsize=16)
        plt.tight_layout()
        plt.show()

    # Take the constraints for the initial and goal configurations.
    # Returns a trajectory for each timestep, the entry has a 3 tuples for each joint
    #   each tuple has 3 elements (q_j^i, dq_j^i, ddq_j^i) st. 0<=j<=2 (joint index), i is the index of the iteration  
    @staticmethod
    def polynomial5(t0, q0, dq0, ddq0, tf, qf, dqf, ddqf, dt=1/100):
        q = lambda a,t: a[0] + a[1]*t + a[2]*(t**2) + a[3]*(t**3) + a[4]*(t**4) + a[5]*(t**5)
        dq = lambda a,t: a[1] + 2*a[2]*t + 3*a[3]*(t**2) + 4*a[4]*(t**3) + 5*a[5]*(t**4)
        ddq = lambda a,t: 2*a[2] + 6*a[3]*t + 12*a[4]*(t**2) + 20*a[5]*(t**3)

        A = np.array([[1, t0, t0**2, t0**3,    t0**4,       t0**5],
                      [0, 1,  2*t0,  3*(t0**2), 4*(t0**3),  5*(t0**4)],
                      [0, 0,  2,     6*t0,      12*(t0**2), 20*(t0**3)],
                      [1, tf, tf**2, tf**3,    tf**4,       tf**5],
                      [0, 1,  2*tf,  3*(tf**2), 4*(tf**3),  5*(tf**4)],
                      [0, 0,  2,     6*tf,      12*(tf**2), 20*(tf**3)]])
        x = []
        for j in range(3):
            b = np.array([[q0[j], dq0[j], ddq0[j], qf[j], dqf[j], ddqf[j]]]).T
            x.append(np.linalg.inv(A) @ b)
        
        traj_all = []
        time = np.linspace(t0, tf, int((tf-t0)/dt))
        for t in time:
            traj = []
            for j in range(3):
                traj.append([q(x[j],t), dq(x[j],t), ddq(x[j],t)])
            traj_all.append(traj)
        
        return np.array(traj_all)

    # Returns a trajectory for each timestep, the entry has a 3 tuples for each joint
    #   each tuple has 3 elements (q_j^i, dq_j^i, ddq_j^i) st. 0<=j<=2 (joint index), i is the index of the iteration  
    @staticmethod
    def PTP(q0, qf, f=10, dq_max=1, ddq_max=10):
        pass

    # Returns a trajectory for each timestep, the entry has a 3 tuples for each joint
    #   each tuple has 3 elements (q_j^i, dq_j^i, ddq_j^i) st. 0<=j<=2 (joint index), i is the index of the iteration  
    @staticmethod
    def LIN(q0, qf, f=10, dq_max=1, ddq_max=10):
        pass

if __name__ == "__main__":
    (q0, qf) = ([-0.5, -0.6, 0], [1.57, 0.5, -2.0])
    t0,tf = 0, 2
    (dq0, dqf) = ([0,0,0], [0,0,0])
    (ddq0, ddqf) = ([0,0,0], [0,0,0])
    traj_poly5 = TrajectoryPlaning.polynomial5(t0, q0, dq0, ddq0, tf, qf, dqf, ddqf)
    TrajectoryPlaning.plot_trajectory(traj=traj_poly5, title="Polynomial - 5th Order")