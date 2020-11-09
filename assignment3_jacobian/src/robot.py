from utils import *

class KUKA_KR10_R1100_2:
    def __init__(self, T_base=None, T_tool=None):
        self.links_dimensions = KUKA_KR10_R1100_2_configs.get_links_dimensions()
        self.joint_limits = KUKA_KR10_R1100_2_configs.get_joints_limits()
        self.T_base = translation_x(0) if T_base is None else T_base
        self.T_tool = translation_x(0) if T_tool is None else T_tool

    def jacobian(self, q, method="skew"):
        from Jacobian import Jacobian
        jacobian = Jacobian(T_base=self.T_base, T_tool=self.T_tool)
        if(method == "skew"):
            return jacobian.calc_skew(q)
        elif(method == "numerical"):
            return jacobian.calc_numerical(q)

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
    robot = KUKA_KR10_R1100_2()
    q = np.zeros((6,1))
    skew = robot.jacobian(q, method="skew")
    numerical = robot.jacobian(q, method="numerical")
    print(skew)
    print(numerical)
    print(calc_error(numerical, skew))