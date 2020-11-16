from utils import *
import visualization as visual
class KUKA_KR10_R1100_2:
    def __init__(self, T_base=None, T_tool=None):
        self.links_dimensions = KUKA_KR10_R1100_2_configs.get_links_dimensions()
        self.joint_limits = KUKA_KR10_R1100_2_configs.get_joints_limits()
        # self.zero_config = []
        self.T_base = translation_x(0) if T_base is None else T_base
        self.T_tool = translation_x(0) if T_tool is None else T_tool


    def print_frame(self, f):
        print(f"Homogeneous Matrix:\n{f}")
        print("Rotation:\n", get_rotation(f))
        print("Position:\n", get_position(f)) 

    def print_frames(self, frames):
        print(f"Note: Frame #{len(frames)-1} -> Tool")
        for i, f in enumerate(frames):
            print(f"Frame #{i}:")
            if(i == len(frames) - 1):
                print(f"--- Tool Frame ---")
            else:
                print(f"--- Joint #{i} Frame ---")
            self.print_frame(f)
            print("-----------")

    def print_angles(self, q):
        for i, angle in enumerate(q):
            print(f"Joint #{i+1}: {angle} rad ---> {angle*180/np.pi} degrees")

    def plot_robot(self, T):
        vis = visual.RobotVisualization_vpython(rate=10, scale=0.0002, radius={"link":0.007, "joint":0.008, "node":0.01, "axe":0.003})
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

    def plot_robot_multi_frames(self, Ts):
        while True:
            for idx, T in enumerate(Ts):
                # print(T)
                vis = visual.RobotVisualization_vpython(rate=1, scale=0.0002, radius={"link":0.007, "joint":0.008, "node":0.01, "axe":0.003})
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


    def inverse_kinematics(self, T, m=-1, plot=True, debug=True):
        from IK import IK

        q, status = IK(T, T_base=self.T_base, T_tool=self.T_tool, m=m, debug=True)

        if(plot == True):
            self.plot_robot(T)    # plot the result
        if(debug == True):
            # self.print_frame(T)    # just print the result in a good way
            self.print_angles(q)
            print(status)

        if(not debug == True):
            # print(q)
            return q

        return q

class KUKA_KR10_R1100_2_configs:
    @staticmethod
    def get_links_dimensions():
        return [400,25,560,25,515,90]

    @staticmethod
    def get_joints_limits():
        deg = [(-170, 170), (-190, 45), (-120, 156), (-185, 185), (-120, 120), (-350, 350)]
        rad = []
        for (i,j) in deg:
            rad.append(((i*np.pi/180), (j*np.pi/180)))
        return rad

if __name__ == "__main__":
    # print(KUKA_KR10_R1100_2_configs.get_joints_limits())
    robot = KUKA_KR10_R1100_2()
    q = np.zeros((6,))
    # q[1] = -np.pi/4
    # q = [0, np.pi/4, 0,    1, 1, 0]
    T = robot.forward_kinematics(q, return_all=True)
    # # robot.print_frame(T)
    # q_calc = robot.inverse_kinematics(T)
    # T_calc = robot.forward_kinematics(q_calc, debug=False)

    # print(q, q_calc)
    # robot.print_frame(T)
    # robot.print_frame(T_calc)

    # print(f"Error using FK then IK then FK -> {calc_error(T, T_calc)}")