from utils import *
import visualization as visual
from utils import translation_x as tx
from utils import translation_y as ty
from utils import translation_z as tz

from utils import rotation_x as rx
from utils import rotation_y as ry
from utils import rotation_z as rz

from utils import dtranslation_x as dtx
from utils import dtranslation_y as dty
from utils import dtranslation_z as dtz

from utils import drotation_x as drx
from utils import drotation_y as dry
from utils import drotation_z as drz

class FANUC_R_2000i_configs:
    # TODO: change static methods to class variables
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

class FANUC_R_2000i:
    def __init__(self, T_base=None, T_tool=None):
        self.num_joints = 6
        self.robot_configs = FANUC_R_2000i_configs
        self.d = self.robot_configs.get_links_dimensions()
        self.joint_limits = self.robot_configs.get_joints_limits()
        self.T_base = translation_x(0) if T_base is None else T_base
        self.T_tool = translation_x(0) if T_tool is None else T_tool
    
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
            print_matrix(f)
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
            if(i == 1): # because of the physical shift without link (if I was using DH it would be much easier)
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
                vis = visual.RobotVisualization_vpython(rate=10, scale=0.0002, radius={"link":0.007, "joint":0.008, "node":0.01, "axe":0.003})
                frame = []
                links = []
                joints = []
                node = get_position(T[-1])
                for i in range(1,len(T)):
                    links.append((get_position(T[i-1]), get_position(T[i])))
                    joints.append(get_position(T[i]))
                for i, l in enumerate(links):
                    if(i == 1): # because of the physical shift without link (if I was using DH it would be much easier)
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

    def get_T_robot_reducible(self, q, pi):
        T_robot = rz(q[0]) @ tx(self.d[1]+pi[0]) @ ty(pi[1]) @ rx(pi[2]) @ ry(q[1]+pi[3]) @ tx(pi[4]) @ rx(pi[5]) @ rz(pi[6]) @ ry(q[2]+pi[7]) @ tx(self.d[5]+pi[8]) @ tz(self.d[4]+pi[9]) @ rz(pi[10]) @ rx(q[3]+pi[11]) @ ty(pi[12]) @ tz(pi[13]) @ rz(pi[14]) @ ry(q[4] + pi[15]) @ tz(pi[16]) @ rz(pi[17]) @ rx(q[5])
        return T_robot
