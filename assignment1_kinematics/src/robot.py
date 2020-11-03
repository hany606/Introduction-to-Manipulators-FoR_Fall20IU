from utils import *
class KUKA_KR10_R1100_2:
    def __init__(self, T_base=None, T_tool=None):
        self.links_dimensions = KUKA_KR10_R1100_2_configs.get_links_dimensions()
        self.joint_limits = KUKA_KR10_R1100_2_configs.get_joints_limits()
        # self.zero_config = []
        self.T_base = translation_x(0) if T_base is None else T_base
        self.T_tool = translation_x(0) if T_tool is None else T_tool


    def print_frame(self, f):
        print(f)
        print("Rotation: ", get_rotation(f))
        print("Position: ", get_position(f)) 

    def print_frames(self, frames):
        print(f"Note: Frame #0 -> World\n Frame #{len(frames)-2} -> End Effector\n Frame #{len(frames)-1} -> Tool")
        for i, f in enumerate(frames):
            print(f"Frame #{i}:")
            if(i == 0):
                print(f"--- World Frame ---")
            elif(i == len(frames) - 2):
                print(f"--- End Effector Frame ---")
            elif(i == len(frames) - 1):
                print(f"--- Tool Frame ---")
            else:
                print(f"--- Joint #{i} Frame ---")
            self.print_frame(f)
            print("-----------")

    def print_angles(self, q):
        for i, angle in enumerate(q):
            print(f"Joint #{i+1}: {angle} rad ---> {angle*180/np.pi} degrees")

    def plot_robot(self, frames):
        pass

    def forward_kinematics(self, q, plot=True, debug=True):
        from FK import FK
        T = FK(q, T_base=self.T_base, T_tool=self.T_tool, return_frames=(plot or debug))
        
        if(plot == True):
            self.plot_robot(T)    # plot the result
        if(debug == True):
            self.print_frames(T)    # just print the result in a good way

        # We need only to return the end-effector
        if(not(plot or debug) == True):
            print(T)    # only end_effector
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
            print(q)
            return q

        return q

class KUKA_KR10_R1100_2_configs:
    @staticmethod
    def get_links_dimensions():
        return [400,25,560,25,515,90]

    @staticmethod
    def get_joints_limits():
        return [(-170, 170), (-190, 45), (-120, 156), (-185, 185), (-120, 120), (-350, 350)]


if __name__ == "__main__":
    robot = KUKA_KR10_R1100_2()
    q = np.zeros((6,))
    q[1] = -np.pi/4
    q = [0, np.pi/4, 0,    1, 1, 0]
    T = robot.forward_kinematics(q, debug=False)
    # robot.print_frame(T)
    q_calc = robot.inverse_kinematics(T)
    T_calc = robot.forward_kinematics(q_calc, debug=False)

    print(q, q_calc)
    robot.print_frame(T)
    robot.print_frame(T_calc)

    print(f"Error using FK then IK then FK -> {calc_error(T, T_calc)}")