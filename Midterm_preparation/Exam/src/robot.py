from utils import *

class KUKA_KR10_R1100_2:
    def __init__(self, T_base=None, T_tool=None):
        self.links_dimensions = KUKA_KR10_R1100_2_configs.get_links_dimensions()
        self.joint_limits = KUKA_KR10_R1100_2_configs.get_joints_limits()
        self.T_base = translation_x(0) if T_base is None else T_base
        self.T_tool = translation_x(0) if T_tool is None else T_tool
    
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


    def jacobian(self, q, method="skew"):
        from Jacobian import Jacobian
        jacobian = Jacobian(T_base=self.T_base, T_tool=self.T_tool)
        if(method == "skew"):
            return jacobian.calc_skew(q)
        elif(method == "numerical"):
            return jacobian.calc_numerical(q)
    
    def check_singularity(self, q, jacobian_method="numerical", singularity_method="rank", debug=True):
        J = self.jacobian(q, method=jacobian_method)
        singularity_flag = False
        u, s, v = np.linalg.svd(J)
        eps = 1e-15

        if(singularity_method == "determinant"):
            if(abs(np.linalg.det(J)) <= eps):
                singularity_flag = True
        
        if(singularity_method == "SVD"):
            if(min(s) <= eps):
                singularity_flag = True
        
        if(singularity_method == "rank"):
            if(np.linalg.matrix_rank(J) < 6):
                singularity_flag = True

        if(debug == True):
            print("Checking Singularity ...")
            print(f"Configuration (q): {q}")
            # print(f"Jacobian (J): {J}")
            print(f"SVD: s: {s}, minimum value: {min(s)}")
            print(f"Determinant (det(J)): {np.linalg.det(J)}")
            print(f"Rank (rank(J)): {np.linalg.matrix_rank(J)}")
            print(f"Result: This configuration is {'a Singular' if singularity_flag == True else 'Not a Singular'}")
        return singularity_flag
class KUKA_KR10_R1100_2_configs:
    @staticmethod
    def get_links_dimensions():
        return [10,5,15,1]

    @staticmethod
    def get_joints_limits():
        deg = [(-350, 350), (-350, 350), (-20, 20), (-350, 350), (-350, 350), (-350, 350)]
        rad = []
        for (i,j) in deg:
            rad.append(((i*np.pi/180), (j*np.pi/180)))
        return rad

if __name__ == "__main__":
    robot = KUKA_KR10_R1100_2()

    q = np.zeros((6,))
    # q[1] = -np.pi/4
    # q = [0, np.pi/4, 0,    1, 1, 0]
    T = robot.forward_kinematics(q, plot=False, return_all=False)
    # # robot.print_frame(T)
    q_calc = robot.inverse_kinematics(T, plot=False, debug=False)
    T_calc = robot.forward_kinematics(q_calc, plot=False, debug=False)

    print(q, q_calc)
    # robot.print_frame(T)
    # robot.print_frame(T_calc)

    print(f"Error using FK then IK then FK -> {calc_error(T, T_calc)}")

    q = np.zeros((6,1))
    skew = robot.jacobian(q, method="skew")
    # numerical = robot.jacobian(q, method="numerical")
    print(skew)
    # print(numerical)
    # print(calc_error(numerical, skew))

