import matplotlib.pyplot as plt
import numpy as np
from math import sqrt, ceil
from utils import get_position, pos2hom

class TrajectoryPlanning:


    @staticmethod
    def plot_trajectory_cartesian(traj, dt=1/100, title="Trajectory", time=None, type=1):
        if(type == 1):
            traj = traj.squeeze()
            p, dp, ddp = traj[:,:, 0], traj[:,:, 1], traj[:,:, 2]
            time = np.linspace(0, dt*len(traj), len(traj)) if time is None else time

            fig, axs = plt.subplots(3,1)
            axs[0].plot(time, p)
            axs[0].set_xlabel("Time - seconds")
            axs[0].set_ylabel("p - m")
            axs[0].legend(["X", "Y", "Z"], loc="upper left", bbox_to_anchor=(1, 1))
            axs[0].set_title("Position")
            
            axs[1].plot(time, dp)
            axs[1].set_xlabel("Time - seconds")
            axs[1].set_ylabel("dp - m/s")
            axs[1].legend(["X", "Y", "Z"], loc="upper left", bbox_to_anchor=(1, 1))
            axs[1].set_title("Velocity")

            axs[2].plot(time, ddp)
            axs[2].set_xlabel("Time - seconds")
            axs[2].set_ylabel("dp - m/s^2")
            axs[2].legend(["X", "Y", "Z"], loc="upper left", bbox_to_anchor=(1, 1))
            axs[2].set_title("Acceleration")

            fig.suptitle(title, fontsize=12)
            plt.tight_layout()
            plt.show()
        # Deprecated
        else:
            p = traj[:,0,0]
            dp = traj[:,0,1]
            time = np.linspace(0, dt*len(traj), len(traj)) if time is None else time

            fig, axs = plt.subplots(2,1)
            axs[0].plot(time, p)
            axs[0].set_xlabel("Time - seconds")
            axs[0].set_ylabel("p - m")
            axs[0].legend(["X", "Y", "Z"], loc="upper left", bbox_to_anchor=(1, 1))
            axs[0].set_title("Position")
            
            axs[1].plot(time, dp)
            axs[1].set_xlabel("Time - seconds")
            axs[1].set_ylabel("dp - m/s")
            axs[1].legend(["X", "Y", "Z"], loc="upper left", bbox_to_anchor=(1, 1))
            axs[1].set_title("Velocity")

            fig.suptitle(title, fontsize=12)
            plt.tight_layout()
            plt.show()

    # Take the trajectory as a parameter
    # Plot 3 figures (Position, Velocity & Acceleration)
    @staticmethod
    def plot_trajectory(traj, dt=1/100, title="Trajectory", time=None):
        traj = traj.squeeze()
        ddq = None
        q, dq = traj[:,:, 0], traj[:,:, 1]
        if(traj.shape[2] == 3):
            ddq = traj[:,:, 2]
        time = np.linspace(0, dt*len(traj), len(traj)) if time is None else time

        fig, axs = None, None
        if(traj.shape[2] == 3):
            fig, axs = plt.subplots(3,1)
        else:
            fig, axs = plt.subplots(2,1)
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
        
        if(traj.shape[2] == 3):
            axs[2].plot(time, ddq)
            axs[2].set_xlabel("Time - seconds")
            axs[2].set_ylabel("dq - rad/sec^2")
            axs[2].legend(["Joint1", "Joint2", "Joint3"], loc="upper left", bbox_to_anchor=(1, 1))
            axs[2].set_title("Acceleration")

        
        fig.suptitle(title, fontsize=12)
        plt.tight_layout()
        plt.show()

    # Take the constraints for the initial and goal configurations.
    # Returns a trajectory for each timestep, the entry has a 3 tuples for each joint
    #   each tuple has 3 elements (q_j^i, dq_j^i, ddq_j^i) st. 0<=j<=2 (joint index), i is the index of the iteration  
    @staticmethod
    def polynomial5(t0, q0, dq0, ddq0, tf, qf, dqf, ddqf, dt=1/1000):
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
    
    @staticmethod
    def _trapezoidal(q0, qf, f=10, dq_max=[1,1,1], ddq_max=[10,10,10], debug=False):
        dt = 1/f
        joints_status = np.empty((3,5))
        synchronization_flag = False    # Has two meaning (one of the profiles for the joints is trapezoidal) and (the case is trapezoidal after synchronization)
        t1 = None
        tau = None
        case = None
        # Check the case and calculate t1 and tau for each joint
        for j in range(3):
            # if(dq_max[j] == 0):
            #     joints_status[j,:] = np.array([-1,0,0, dq_max[j], ddq_max[j]])
            #     continue
            # joints_status.append([])
            delta_q = abs(qf[j] - q0[j])
            dq_max_dash = sqrt(delta_q*ddq_max[j])
            dq_max_tmp = dq_max[j]
            tau = None
            t1 = None
            c = None
            if(dq_max_dash <= dq_max[j]):
                t1 = sqrt(delta_q/ddq_max[j])
                tau = 0
                c = 2#"Triangular"  
                dq_max_tmp = dq_max_dash              
            else:
                t1 = dq_max[j]/ddq_max[j]
                tau = delta_q/dq_max[j]
                c = 1#"Trapezoidal"
                synchronization_flag = True
            joints_status[j,:] = np.array([c,t1,tau, dq_max_tmp, ddq_max[j]])

        # Synhronize and select t1, tau and case for all the joints synchronized
        if(synchronization_flag):
            # It will Trapezoidal profile for all
            case = 1#"Trapezoidal"
        else:
            # All of them are triangular and it will be triangular for all
            case = 2#"Triangular"
        if(debug):
            print(f"After Selecting profiles: \n{joints_status}")
        t1_dash = np.max(joints_status[:,1])
        tau_dash = np.max(joints_status[:,2] - joints_status[:,1]) + t1_dash
        # Replan after synchronization
        joints_status_dash = np.empty((3,5))
        for j in range(3):
            # if(dq_max[j] == 0):
            #     joints_status[j,:] = np.array([-1,0,0, dq_max[j], ddq_max[j]])
            #     continue
            delta_q = abs(q0[j] - qf[j])
            dq_max_2dash = None
            ddq_max_2dash = None
            # Triangular
            if(synchronization_flag == False):
                dq_max_2dash = delta_q / t1_dash
                ddq_max_2dash = delta_q / (t1_dash*t1_dash)       
            # Trapezoidal
            else:
                dq_max_2dash = delta_q / tau_dash
                ddq_max_2dash = delta_q / (tau_dash*t1_dash)  
            joints_status_dash[j,:] = np.array([case, t1_dash, tau_dash, dq_max_2dash, ddq_max_2dash])
        if(debug):
            print(f"After Synchroniztion: \n{joints_status_dash}")
        # joints_status = np.array(joints_status_dash)
        # Discretecize
        # Get n, m
        n = ceil(t1_dash/dt)
        m = ceil((tau_dash - t1_dash)/dt) 
        # Caluclate t1, tau
        t1_2dash = n*dt
        tau_2dash = m*dt + t1_2dash
    
        # Calculate dq_max_3dash, ddq_max_3dash
        joints_status_2dash = np.empty((3,5))
        for j in range(3):
            # if(dq_max[j] == 0):
            #     joints_status[j,:] = np.array([-1,0,0, dq_max[j], ddq_max[j]])
            #     continue
            delta_q = abs(q0[j] - qf[j])
            dq_max_3dash = None
            ddq_max_3dash = None
            # Triangular
            if(synchronization_flag == False):
                dq_max_3dash = delta_q / t1_2dash
                ddq_max_3dash = delta_q / (t1_2dash*t1_2dash)       
            else:
                dq_max_3dash = delta_q / tau_2dash
                ddq_max_3dash = delta_q / (tau_2dash*t1_2dash)  
            joints_status_2dash[j,:] = np.array([case, t1_2dash, tau_2dash, dq_max_3dash, ddq_max_3dash])
        if(debug):
            print(f"After Discretecization: \n{joints_status_2dash}")

        # Apply the trajectory equations
        total_time = None
        # if trapezoidal
        if(synchronization_flag):
            total_time = (t1_2dash+tau_2dash)
        # if triangular
        else:
            total_time = t1_2dash*2
        if(debug):
            print(f"t1 = {t1_2dash}, tau = {tau_2dash}")
            print(f"Total time -> {total_time}s")
        num_timesteps = 1000    # Number of steps for the simulation time
        time = np.linspace(0, total_time+dt*2, num_timesteps)    # simulation time not control time
        traj = np.empty((num_timesteps,3,3))
        pos_prev = q0
        direction = [np.sign(qf[j] - q0[j]) for j in range(3)]
        if(debug):
            if(joints_status_2dash[0][0] == 1):
                print("Trapezoidal Profile")
            else:
                print("Triangular Profile")
        for i,t in enumerate(time):
            for j in range(3):
                acc, vel, pos = 0, 0, 0
                # if(dq_max[j] == 0):
                #     traj[i][j] = np.array([pos_prev[j], vel, acc])
                #     continue
                # Apply formulas
                # Trapezoidal profile
                if(joints_status_2dash[j][0] == 1):
                    # Accelerate
                    if(0 <= t and t < t1_2dash):
                        # print(f"1 -> {t}")
                        acc = +joints_status_2dash[j][4]*direction[j]
                        vel = acc*t
                        pos = pos_prev[j] + vel*(time[1] - time[0])
                    # Zero Acceleration
                    elif(t1_2dash <= t and t < tau_2dash):
                        acc = 0
                        vel = joints_status_2dash[j][3]*direction[j]
                        pos = pos_prev[j] + vel*(time[1] - time[0])
                    # Deceleration
                    elif(tau_2dash <= t and t <= total_time):
                        acc = -joints_status_2dash[j][4]*direction[j]
                        vel = direction[j]*joints_status_2dash[j][3] + acc*(t-tau_2dash)
                        pos = pos_prev[j] + vel*(time[1] - time[0])
                    else:
                        acc = 0
                        vel = 0
                        pos = pos_prev[j]

                # Triangular profile
                else:
                    # Accelerate
                    if(0 <= t and t < t1_2dash):
                        acc = +joints_status_2dash[j][4]*direction[j]
                        vel = acc*t
                        pos = pos_prev[j] + vel*(time[1] - time[0])

                    # Deceleration
                    elif(t1_2dash <= t and t <= total_time):
                        acc = -joints_status_2dash[j][4]*direction[j]
                        vel = direction[j]*joints_status_2dash[j][3] + acc*(t-t1_2dash)
                        pos = pos_prev[j] + vel*(time[1] - time[0])
                    
                    else:
                        acc = 0
                        vel = 0
                        pos = pos_prev[j]

                traj[i][j] = np.array([pos, vel, acc])
                pos_prev[j] = pos
        return traj, time

    # Performs PTP command in robotics manipulators (Point to Point) (Joint space trajectory planning)
    # Returns a trajectory for each timestep, the entry has a 3 tuples for each joint
    #   each tuple has 3 elements (q_j^i, dq_j^i, ddq_j^i) st. 0<=j<=2 (joint index), i is the index of the iteration  
    @staticmethod
    def PTP(q0, qf, f=10, dq_max=[1,1,1], ddq_max=[10,10,10], debug=False):
        return TrajectoryPlanning._trapezoidal(q0, qf, f, dq_max, ddq_max, debug)

    # Performs LIN command on in robotics manipulators (Move in linear trajectory from point to point) (Cartesian space trajectory planning)
    # Returns a trajectory for each timestep, the entry has a 3 tuples for each joint
    #   each tuple has 3 elements (q_j^i, dq_j^i, ddq_j^i) st. 0<=j<=2 (joint index), i is the index of the iteration  
    @staticmethod
    def LIN(robot, p0, pf, f=10, dp_max=[1,1,1], ddp_max=[10,10,10], debug=False):
        traj, time = TrajectoryPlanning._trapezoidal(p0, pf, f, dp_max, ddp_max, debug)
        traj = traj.squeeze()
        p, dp, ddp = traj[:,:, 0], traj[:,:, 1], traj[:,:, 2]
        joint_traj = np.empty((traj.shape[0], 3, 2))
        for i in range(traj.shape[0]):
            q = robot.inverse_kinematics(pos2hom(p[i]), plot=False, debug=False, debug_status=False)
            J = robot.jacobian(q, method="numerical")
            dq= (np.linalg.pinv(J)[:, :3] @ dp[i])
            for j in range(3):
                joint_traj[i][j] = np.array([q[j], dq[j]])
        return traj, time, joint_traj
    
    # Deprecated
    @staticmethod
    def LIN1(robot, p0, pf, f=10, dp_max=[1,1,1], ddp_max=[10,10,10], num_samples=100, debug=False):
        def pos2hom(pos):
            hom = np.zeros((4,4))
            hom[:3,3] = pos.T
            hom[3,3] = 1
            return hom

        p0, pf = np.array(p0), np.array(pf)
        q0, status = robot.inverse_kinematics(pos2hom(p0), plot=False, debug=False, debug_status=True)
        # Create equation of line using the two points
        line_dir = pf - p0
        line_eq = lambda a: p0 + a*line_dir
        # (Sampling) Split the path to sub points
        sample = []
        for i in range(num_samples):
            sample.append(line_eq((i+1)/num_samples))
            # print(line_eq((i+1)/num_samples))
        # For each point in the sample
        joint_traj = []
        total_time = 0
        cartesian_traj = []
        for i,point in enumerate(sample):
            # Calculate the joints positions using IK
            point = np.array(point.copy()).reshape((3,1))
            # ----------------- PTP Planning --------------------------------
            qi_ref, status = robot.inverse_kinematics(pos2hom(point), plot=False, debug=False, debug_status=True)
            # Plan point to point using the sub points
            J = robot.jacobian(qi_ref, method="numerical")
            dq_max = (np.linalg.pinv(J) @ np.array([dp_max[0], dp_max[1], dp_max[2], dp_max[2], dp_max[2], dp_max[2]]).T)[:3]
            ddq_max = ddp_max
            traj_ptp, time = TrajectoryPlanning.PTP(q0.copy(), qi_ref.copy(), f=f, dq_max=dq_max, ddq_max=ddq_max, debug=debug)
            total_time += time[-1]
            # Get the final reached configuration (joints' positions and velocities) (It should be the reference point) from the generated trajectory
            qis = traj_ptp[:,:,0]#[-1,:,0]
            qi = qis[-1]
            dqis = traj_ptp[:,:,1]
            dqi = dqis[-1]
            if(debug):
                print(f"## {i}th Sample:\nInitial joint configuration: {q0}\nGoal joint configuration: {qi_ref}\nReached joint configruation: {qi}")
                print(f"Taken time to reach such joint configuration: {time[-1]}")
                print(f"Total time till now: {total_time}")
            # --------------------------------------------------------------
            # Get the final reached point (It should be the reference point) from the generated trajectory
            pi_hom = robot.forward_kinematics(qi, plot=False, debug=False)
            pi = get_position(pi_hom)
            if(debug):
                print(f"Goal (Final) {qi_ref}\nReal (Final): {qi}")
                print(pi)
            # Each pi in the sample consist of joint trajectory planning which has number of points in the trajectory
            # Calculate the cartesian velocities using Jacobian
            joints_traj_select_indx = np.random.choice(qis.shape[0], int(1/num_samples*qis.shape[0]), replace=False)
            joints_traj_select_indx.sort()
            q0 = qi
            p0 = pi
            # cartesian_traj.append(p0)
            q_ptp_selected = qis[joints_traj_select_indx]
            dq_ptp_selected = dqis[joints_traj_select_indx]
            joint_traj.extend(q_ptp_selected)
            for i in range(len(joints_traj_select_indx)):
                cartesian_traj.append([])
                p = get_position(robot.forward_kinematics(q_ptp_selected[i], plot=False, debug=False))
                J = robot.jacobian(q_ptp_selected[i], method="skew")
                dp = J @ dq_ptp_selected[i].T
                cartesian_traj[-1].append([p, dp[:3]])
        cartesian_traj = np.array(cartesian_traj)
        joint_traj = np.array(joint_traj)
        print(f"Goal (Final) {pf}\nReal (Final): {p0}")
        return cartesian_traj, joint_traj

if __name__ == "__main__":
    from robot import RRR_robot
    robot = RRR_robot()
    # ----------- Task 1 -----------------------------
    # ----------- Task 2 -----------------------------
    print("--------------------- Polynomial 5th ordedr ---------------------")
    (q0, qf) = ([-0.5, -0.6, 0], [1.57, 0.5, -2.0])
    t0,tf = 0, 2
    (dq0, dqf) = ([0,0,0], [0,0,0])
    (ddq0, ddqf) = ([0,0,0], [0,0,0])
    traj_poly5 = TrajectoryPlanning.polynomial5(t0, q0, dq0, ddq0, tf, qf, dqf, ddqf)
    TrajectoryPlanning.plot_trajectory(traj=traj_poly5, title="Polynomial - 5th Order")

    # Ts = []
    # trail = []
    # for traj in traj_poly5[:,:,0]:
    #     q = traj
    #     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    #     Ts.append(T)
    #     trail.append(get_position(T[-1]))
    #     # print(trail[-1])
    # robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)


    print("--------------------- PTP ---------------------")
    (q1, q2) = ([0,0,0], [-0.9,-2.3,1.2])
    f = 10
    dq_max = [1, 1, 1]
    ddq_max = [10, 10, 10]
    traj_ptp, time = TrajectoryPlanning.PTP(q1.copy(), q2.copy(), f, dq_max, ddq_max, debug=True)
    print(f"Setpoints: Starting {q1}, Final {q2}")
    print(f"Goal (Final) {q2}\nReal (Final): {traj_ptp[-1,:,0]}")
    TrajectoryPlanning.plot_trajectory(traj=traj_ptp, title="PTP - Trapezoidal", time=time)


    # Ts = []
    # trail = []
    # for traj in traj_ptp[:,:,0]:
    #     q = traj
    #     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    #     Ts.append(T)
    #     trail.append(get_position(T[-1]))
    #     # print(trail[-1])
    # robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)


    print("--------------------- LIN ---------------------")
    (p1, p2) = ([1,0,2], [1/sqrt(2),1/sqrt(2),1.2])
    f = 10
    dp_max = [1, 1, 1]
    ddp_max = [10, 10, 10]
    traj_lin, time, joint_traj = TrajectoryPlanning.LIN(robot, p1.copy(), p2.copy(), f, dp_max, ddp_max)
    print(f"Setpoints: Starting {p1}, Final {p2}")
    print(f"Goal (Final) {p2}\nReal (Final): {traj_lin[-1,:,0]}")
    TrajectoryPlanning.plot_trajectory_cartesian(traj=traj_lin, title="2nd version - LIN - Trapezoidal - Cartesian Space", time=time)
    TrajectoryPlanning.plot_trajectory(traj=joint_traj, title="2nd version LIN - Trapezoidal - Joint Space", time=time)

    # Ts = []
    # trail = []
    # for traj in joint_traj[:,:,0]:
    #     q = traj
    #     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    #     Ts.append(T)
    #     trail.append(get_position(T[-1]))
    #     # print(trail[-1])
    # robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)


    # All
    # Ts = []
    # trail = []
    # for traj in traj_poly5[:,:,0]:
    #     q = traj
    #     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    #     Ts.append(T)
    #     trail.append(get_position(T[-1]))
    # for traj in traj_ptp[:,:,0]:
    #     q = traj
    #     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    #     Ts.append(T)
    #     trail.append(get_position(T[-1]))
    # for traj in joint_traj[:,:,0]:
    #     q = traj
    #     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    #     Ts.append(T)
    #     trail.append(get_position(T[-1]))
    # robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/10, trail=trail)

    print("1st Version: ")
    num_samples = 2
    traj_lin = TrajectoryPlanning.LIN1(robot, p1.copy(), p2.copy(), f, dp_max, ddp_max, num_samples=num_samples)
    cartesian_traj = traj_lin[0]
    joint_traj = traj_lin[1]
    print(f"Setpoints: Starting {p1}, Final {p2}")
    print(f"Goal (Final) {p2}\nReal (Final): {cartesian_traj[-1,0,0]}")
    TrajectoryPlanning.plot_trajectory_cartesian(traj=cartesian_traj, title="1st version - LIN - Cartesian - Trapezoidal", type=2)

    Ts = []
    trail = []
    for traj in joint_traj:
        q = traj
        T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
        Ts.append(T)
        trail.append(get_position(T[-1]))
        # print(trail[-1])
    robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)

    # print("2nd Version")
