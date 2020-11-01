class KUKA_KR10_R1100_2:
    def __init__(self):
        self.links = [400,25,560,25,515,90]
        self.joint_limits = [(-170, 170), (-190, 45), (-120, 156), (-185, 185), (-120, 120), (-350, 350)]
        # self.zero_config = []