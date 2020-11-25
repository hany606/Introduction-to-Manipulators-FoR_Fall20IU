from utils import *
import visualization as visual
from TrajectoryPlanning import TrajectoryPlanning


class RRR_robot:
    def __init__(self, T_base=translation_x(0), T_tool=translation_x(0)):
        self.links_dimensions = RRR_robot_configs.get_links_dimensions()
        self.joint_limits = RRR_robot_configs.get_joints_limits()
        self.T_base = T_base
        self.T_tool = T_tool
        self.visualization_radius = {"link":0.003, "joint":0.004, "node":0.004, "axe":0.003, "trajectory_trail": 0.0005}
        self.visualization_scale = 0.05
    
    def print_frames(self, frames):
        print(f"Note: Frame #{len(frames)-1} -> Tool")
        for i, f in enumerate(frames):
            print(f"Frame #{i}:")
            if(i == len(frames) - 1):
                print(f"--- Tool Frame ---")
            else:
                print(f"--- Joint #{i} Frame ---")
            print_matrix(f)
            print("-----------")


    def print_angles(self, q):
        for i, angle in enumerate(q):
            print(f"Joint #{i}: {angle} rad ---> {angle*180/np.pi} degrees")

    def plot_robot(self, T):
        vis = visual.RobotVisualization_vpython(rate=10, scale=self.visualization_scale, radius=self.visualization_radius)
        frame = []
        links = []
        joints = []
        node = get_position(T[-1])
        for i in range(1,len(T)):
            links.append((get_position(T[i-1]), get_position(T[i])))
            joints.append(get_position(T[i]))
        for i, l in enumerate(links):
            # i == (shift_link-1)
            if(i == 0): # because of the physical shift without link (if I was using DH it would be much easier)
                p1 = l[0]
                p2 = l[1]
                # print(p2)
                p1_1 = p1.copy()
                p2_1 = p2.copy()
                p2_2 = p2.copy()
                p2_1[0] = 0.0
                p1_2 = p2_1.copy()
                frame.append(["link", p1_1, p2_1])
                frame.append(["link", p1_2, p2_2])
                continue
            frame.append(["link", l[0], l[1]])
        for j in joints:
            frame.append(["joint", j])
        frame.append(["node", node])
        frame.append(["time", 0, 0])
        # print(frame)
        while True:
            vis.render_frame(frame, axis=False)

    def plot_robot_multi_frames(self, Ts, trail=None, rate_factor=1):
        vis = visual.RobotVisualization_vpython(rate=1*rate_factor, scale=self.visualization_scale, radius=self.visualization_radius)
        while True:
            for idx, T in enumerate(Ts):
                # print(T)
                frame = []
                links = []
                joints = []
                node = get_position(T[-1])
                for i in range(1,len(T)):
                    links.append((get_position(T[i-1]), get_position(T[i])))
                    joints.append(get_position(T[i]))
                for i, l in enumerate(links):
                    # i == (shift_link-1)
                    if(i == 0): # because of the physical shift without link (if I was using DH it would be much easier)
                        p1 = l[0]
                        p2 = l[1]
                        # print(p2)
                        p1_1 = p1.copy()
                        p2_1 = p2.copy()
                        p2_2 = p2.copy()
                        p2_1[0] = 0.0
                        p1_2 = p2_1.copy()
                        frame.append(["link", p1_1, p2_1])
                        frame.append(["link", p1_2, p2_2])
                        continue
                    frame.append(["link", l[0], l[1]])
                for j in joints:
                    frame.append(["joint", j])
                frame.append(["node", node])
                frame.append(["time", idx, 0])
                if(trail is not None):
                    frame.append(["trajectory_trail", trail[idx]])
                # print(frame)
                vis.render_frame(frame, axis=False)

    def forward_kinematics(self, q, plot=True, debug=True, return_all=False):
        from FK import FK
        T = FK(q, T_base=self.T_base, T_tool=self.T_tool, return_frames=(plot or debug or return_all))
        
        if(debug == True):
            self.print_frames(T)    # just print the result in a good way
        if(plot == True):
            self.plot_robot(T)    # plot the result
        if(return_all == True):
            return T
        # We need only to return the end-effector
        if(not(plot or debug) == True):
            # print(T)    # only end_effector
            return T
        return T[-1]    # end_effector

    def inverse_kinematics(self, T, m=-1, plot=True, debug=True, debug_status=False):
        from IK import IK

        q, status = IK(T, T_base=self.T_base, T_tool=self.T_tool, m=m, debug=True)

        if(plot == True):
            self.plot_robot(T)    # plot the result
        if(debug == True):
            # self.print_frame(T)    # just print the result in a good way
            self.print_angles(q)
            print(status)

        if(debug_status == True):
            return q, status
        
        return q


    def jacobian(self, q, method="skew"):
        from Jacobian import Jacobian
        jacobian = Jacobian(T_base=self.T_base, T_tool=self.T_tool)
        if(method == "skew"):
            return jacobian.calc_skew(q)
        elif(method == "numerical"):
            return jacobian.calc_numerical(q)
    
    def hello(self):
        print("elfds")
        return 0
    def check_singularity(self, q, jacobian_method="numerical", singularity_method="rank", debug=True):
        J = self.jacobian(q, method=jacobian_method)
        singularity_flag = False
        u, s, v = np.linalg.svd(J)
        eps = 1e-15

        if(J.shape[1] == J.shape[0] and singularity_method == "determinant"):
            if(abs(np.linalg.det(J)) <= eps):
                singularity_flag = True
        
        if(singularity_method == "SVD"):
            if(min(s) <= eps):
                singularity_flag = True
        
        if(singularity_method == "rank"):
            if(np.linalg.matrix_rank(J) < 3):
                singularity_flag = True

        if(debug == True):
            print("Checking Singularity ...")
            print(f"Configuration (q): {q}")
            # print(f"Jacobian (J): {J}")
            print(f"SVD: s: {s}, minimum value: {min(s)}")
            if(J.shape[1] == J.shape[0]):
                print(f"Determinant (det(J)): {np.linalg.det(J)}")
            else:
                print("Dimension are not equal to each other, it is impossible to calculate the determinant")
            print(f"Rank (rank(J)): {np.linalg.matrix_rank(J)}")
            print(f"Result: This configuration is {'a Singular' if singularity_flag == True else 'Not a Singular'}")
        return singularity_flag

    def polynomial5(self, t0, q0, dq0, ddq0, tf, qf, dqf, ddqf, dt=1/100):
        return TrajectoryPlanning.polynomial5(t0, q0, dq0, ddq0, tf, qf, dqf, ddqf)
    
    def PTP(self, q0, qf, f=10, dq_max=1, ddq_max=10):
        traj_ptp, time = TrajectoryPlanning.PTP(q0.copy(), qf.copy(), f, dq_max, ddq_max)
        return traj_ptp
        
    def LIN(self, p0, pf, f=10, dp_max=1, ddp_max=10, num_samples=100, debug=False):
        return TrajectoryPlanning.LIN(self, p0.copy(), pf.copy(), f, dp_max, ddp_max, num_samples=num_samples, debug=debug)
class RRR_robot_configs:
    @staticmethod
    def get_links_dimensions():
        return [1,1,1]

    @staticmethod
    def get_joints_limits():
        deg = [(-350, 350), (-180, 180), (-180, 180)]
        rad = []
        for (i,j) in deg:
            rad.append(((i*np.pi/180), (j*np.pi/180)))
        return rad

if __name__ == "__main__":
    robot = RRR_robot()
    q = np.zeros((3,1))
    skew = robot.jacobian(q, method="skew")
    numerical = robot.jacobian(q, method="numerical")
    print(skew)
    print(numerical)
    print(f"Erro between Numerical and Skew for jacobian -> {calc_error(numerical, skew)}")

    q = np.zeros((3,))
    # q[1] = -np.pi/4
    # q = [0, np.pi/4, 0,    1, 1, 0]
    T = robot.forward_kinematics(q, plot=False, return_all=False)
    # # robot.print_frame(T)
    q_calc = robot.inverse_kinematics(T, plot=False)
    # # T_calc = robot.forward_kinematics(q_calc, debug=False)

    # # print(q, q_calc)
    # # robot.print_frame(T)
    # # robot.print_frame(T_calc)

    # # print(f"Error using FK then IK then FK -> {calc_error(T, T_calc)}")

    (p1, p2) = ([1,0,2], [1/np.sqrt(2),1/np.sqrt(2),1.2])
    f = 10
    dp_max = 1
    ddp_max = 10
    print(f"Setpoints: Starting {p1}, Final {p2}")
    traj_lin = robot.LIN(p1.copy(), p2.copy(), f, dp_max, ddp_max)