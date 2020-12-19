import scipy.io
import numpy as np
import visualization as visual
import time
from math import sqrt
from Jacobian import Jacobian
from utils import *
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

# from tqdm import tqdm

class Calibration:
    def __init__(self, robot,
                       dataset_file="calibration_dataset.mat",
                       num_configs=24, num_samples=10, num_reference_points=3,
                       reference_points_tags=["mA", "mB", "mC"],
                       save_postfix=""):
        self.dataset_file = dataset_file
        self.num_configs = num_configs
        self.num_samples = num_samples
        self.num_reference_points = num_reference_points
        self.reference_points_tags = reference_points_tags
        self.num_joints = robot.num_joints
        self.dimension = 3
        self.num_unknown_parameters = 4+8+6 # Only the robot # 27: including T_base and T_tool
        self.robot = robot
        self.save_postfix=save_postfix
        
        self.jacobian = Jacobian(self.robot)
                
        self.d = robot.robot_configs.get_links_dimensions()

        # self.pi_0 = np.zeros((self.num_unknown_parameters, 1))
        self.pi_0 = np.array([self.d[1], 0,0,0,0,0,0,0, self.d[5], self.d[4], 0, 0,0,0,0,0,0,0], dtype=np.float).reshape((self.num_unknown_parameters, 1)) 
        
        self.visualization_radius = {"node":0.003}
        self.visualization_scale = 0.1
        self.visualization_rate = 5

        
        self.read_mat_file()
        self.splitter()
        
    def get_dataset_raw(self):
        return self.dataset_raw
    
    def read_mat_file(self):
        self.dataset_raw = scipy.io.loadmat(self.dataset_file)
        
    
    def _rescale(self, vec):
        return [vec[0]*1000, vec[1]*1000, vec[2]*1000]
    def splitter(self):
        self.configruations = np.empty((self.num_configs, self.num_samples, self.num_joints, 1))
        self.dataset = np.empty((self.num_configs, self.num_samples, self.num_reference_points, self.dimension))
        for i in range(self.num_configs):
            for j in range(self.num_samples):
                self.configruations[i, j] = np.array(self.dataset_raw["q"][j+self.num_samples*i]).reshape((self.num_joints, 1))
                for k in range(self.num_reference_points):
                    original_scale = self.dataset_raw[self.reference_points_tags[k]][j+self.num_samples*i]
                    self.dataset[i, j, k] = self._rescale(original_scale)
                    # print(f"{i}, {j}, {k} <- {self.reference_points_tags[k]}{j+self.num_samples*i}")
        # return(self.dataset[0,0])

    def get_std_config(self, config=None, config_idx=None):
        if(config_idx is not None):
            config = self.dataset[config_idx]
        if(config is None):
            raise AttributeError('Please provide parameter config or config_idx')
        std = []
        # print(config[:,0]) # 10 repetitions for mA in the configuration
        for i in range(self.num_reference_points):
            std.append(np.std(config[:,i], axis=0))
        return std
        
    def get_mean_config(self, config=None, config_idx=None):
        if(config_idx is not None):
            config = self.dataset[config_idx]
        if(config is None):
            raise AttributeError('Please provide parameter config or config_idx')
        mean = []
        for i in range(self.num_reference_points):
            mean.append(np.mean(config[:,i], axis=0))
        return mean      

    def visualize(self):
        vis = visual.RobotVisualization_vpython(rate=self.visualization_rate,
                                                scale=self.visualization_scale,
                                                radius=self.visualization_radius)
        frame = []
        for i, config in enumerate(self.dataset):
            # print(f"{i+1}th Configuration")
        # for i in range(len(self.dataset)):
        # config = self.dataset[0]
            for sample in config:
                for pos in sample:       
                    frame.append(["node", pos])
                # print(f"Std for mA, mB, mC: {self.get_std_config(config_idx=0)}")
                # print(f"Mean: {self.get_mean_config(config_idx=0)}")
            # print(f"Std for mA, mB, mC:\n{self.get_std_config(config=config)}")
            # print(f"Mean:\n{self.get_mean_config(config=config)}")
            # print("----------------------------------------------------------------")

        while True:
            vis.render_frame(frame, axis=True)
            # time.sleep(5)
            
            
    # Use q, pi (error) in order to base and tool tranformations             
    def _step1(self):                    
        sum1 = np.zeros((6+3*self.num_reference_points,6+3*self.num_reference_points))
        sum2 = np.zeros((6+3*self.num_reference_points,1))
        # sum
        # print(sum2)
        for i in range(self.num_configs):
            for j in range(self.num_samples):
                delta_p_i = np.empty((3*self.num_reference_points,1))
                q = self.configruations[i, j].copy()
                self.T_robot = self.robot.get_T_robot_reducible(q, self.pi)
                p_robot_i = get_position(self.T_robot).copy().reshape((3,1))
                R_robot_i = get_rotation(self.T_robot).copy().reshape((3,3))
                # Calculate A matrix
                A = np.empty((3*self.num_reference_points, 6+3*self.num_reference_points))
                for k in range(self.num_reference_points):
                    A[k*3:k*3+3, :3] = np.eye(3)
                    A[k*3:k*3+3, 3:6] = skew(p_robot_i).T
                    for kk in range(self.num_reference_points):
                        if(kk == k):
                            A[k*3:k*3+3, 6+kk*3:6+kk*3+3] = R_robot_i

                        else:
                            A[k*3:k*3+3, 6+kk*3:6+kk*3+3] = np.zeros((3,3))
                    # print(self.dataset[i,j,k])
                    # print(get_position(self.T_base@self.T_robot@self.T_tool[k]))
                    delta_p_i_j = self.dataset[i,j,k].copy() - get_position(self.T_robot).copy()
                    delta_p_i[k*3:k*3+3] = delta_p_i_j.reshape(3,1)
                # Calculate the 1st summation
                sum1 += A.T @ A
                # Calulate the 2nd summation
                sum2 += A.T @ delta_p_i
                
        # Apply the formula and get p_base, phi_base, u_tool^1, ..., u_tool^num_reference_points
        tmp = np.linalg.inv(sum1) @ sum2  # shape: ((6+3n)x1)
        p_base = np.array(tmp[:3, 0]).reshape((3,1))
        phi_base = np.array(tmp[3:6, 0]).reshape((3,1))
        # From p_base and phi_base get T_base
        R_base = skew(phi_base) + np.eye(3)
        T_base = get_homogenous(R_base, p_base)
        T_tool = np.empty((self.num_reference_points, 4, 4))
        # From u_tool^j get p_tool^j st. j=num_reference_points
        # From p_tool^j get T_tool^j -> get T_tool
        for i in range(self.num_reference_points):
            u_tool_j = np.array(tmp[6+i*3:6+(i+1)*3, 0]).reshape((3,1))
            p_tool_j = R_base.T @ u_tool_j
            T_tool_j = get_homogenous(np.eye(3), p_tool_j)
            T_tool[i] = T_tool_j
        return T_base, T_tool
    
    # Use T_base, T_tool, q, pi
    def _step2(self):
        sum1 = np.zeros((self.num_unknown_parameters,self.num_unknown_parameters))
        sum2 = np.zeros((self.num_unknown_parameters,1))
        for i in range(self.num_configs):
            for j in range(self.num_samples):
                q = self.configruations[i, j].copy()
                self.T_robot = self.robot.get_T_robot_reducible(q, self.pi)
                # Calculate identification Jacobian J_pi
                for k in range(self.num_reference_points):
                    jacobian_pi = self.jacobian.calc_identification_jacobian(T_base=self.T_base, T_tool=self.T_tool[k], q=q, pi=self.pi, pi_0=self.pi_0)
                    jacobian_pi_jp = jacobian_pi[:3].reshape((3, self.num_unknown_parameters))
                    delta_p_i_j = (self.dataset[i,j,k].copy() - get_position(self.T_robot).copy()).reshape((3,1))
                    
                    # Calculate the 1st summation
                    sum1 += jacobian_pi_jp.T @ jacobian_pi_jp
                    # Calculate the 2nd summation
                    sum2 += jacobian_pi_jp.T @ delta_p_i_j
                    
        # Apply the formula and get delta_pi
        delta_pi = np.linalg.inv(sum1) @ sum2
        
        return delta_pi
    
    def _terminamtion_criteria(self, delta_pi, epsilon):
        stopping_sum = 0
        for i in range(self.num_configs):
            for j in range(self.num_samples):
                q = self.configruations[i, j].copy()
                self.T_robot = self.robot.get_T_robot_reducible(q, self.pi)
                # Calculate identification Jacobian J_pi
                for k in range(self.num_reference_points):
                    jacobian_pi = self.jacobian.calc_identification_jacobian(T_base=self.T_base, T_tool=self.T_tool[k], q=q, pi=self.pi, pi_0=self.pi_0)
                    jacobian_pi_jp = jacobian_pi[:3].reshape((3, self.num_unknown_parameters))
                    delta_p_i_j = (self.dataset[i,j,k].copy() - get_position(self.T_robot).copy()).reshape((3,1))
                    term = jacobian_pi_jp.dot(delta_pi) - delta_p_i_j
                    stopping_sum += term.T @ term
        print(np.linalg.norm(stopping_sum))
        if(np.linalg.norm(stopping_sum) < epsilon):
            return True
        return False
    
    def calibrate(self, max_num_steps=101, alpha=0.001, epsilon=1e-8):
        steps = 0
        # define pi
        self.pi = self.pi_0
        self.T_base = np.eye(4)
        self.T_tool = np.zeros((self.num_reference_points, 4, 4))
        for i in range(self.num_reference_points):
            self.T_tool[i,:,:] = np.eye(4)
        while True:
            print(f"{steps+1}th iteration:")
            # return
            self.T_base, self.T_tool = self._step1()
            # print("T_base")
            # print(self.T_base)
            # print("T_tool")
            # print(self.T_tool)
            delta_pi = self._step2()
            self.pi += alpha*delta_pi

            if(steps % 10 == 0):
                print(f"Pi:\n{self.pi}")
                np.save(f"pi{self.save_postfix}.npy", self.pi)
                print(f"T_base:\n{self.T_base}")
                np.save(f"T_base{self.save_postfix}.npy", self.T_base)
                print(f"T_tool:\n{self.T_tool}")
                np.save(f"T_tool{self.save_postfix}.npy", self.T_tool)
            
            steps += 1
            if(steps >= max_num_steps or self._terminamtion_criteria(delta_pi, epsilon)):
                break
            
        print("-------------------------")
        print(self.pi)
        
    def RMS_report(self, pi=None, T_base=None, T_tool=None):
        pi = self.pi if pi is None else pi
        T_base = self.T_base if T_base is None else T_base
        T_tool = self.T_tool if T_tool is None else T_tool
        max_dist = 0
        error = 0 #sqrt(1/n sum(^2))
        error_coordinates = np.zeros(self.dimension)
        max_err_coordinates = np.zeros(self.dimension)
        for i in range(self.num_configs):
            for j in range(self.num_samples):
                q = self.configruations[i, j].copy()
                T_robot = self.robot.get_T_robot_reducible(q, pi)
                for k in range(self.num_reference_points):
                    T = T_base @ T_robot @ T_tool[k]
                    new_pos = get_position(T).copy()
                    pos = self.dataset[i, j, k].copy()
                    # print(new_pos - pos)

                    dist = np.linalg.norm(new_pos - pos)
                    max_dist = max(max_dist, dist)
                    error += dist**2
                    for e in range(self.dimension):
                        err = abs(new_pos[e] - pos[e])
                        error_coordinates[e] += err**2
                        max_err_coordinates[e] = max(max_err_coordinates[e], err)
        N = (self.num_configs*self.num_samples*self.num_reference_points)
        error = sqrt(error/N)
        print(f"RMS Distance Error: {error}")
        print(f"Max Distance error (mm): {max_dist}")
        print("-------------------------")
        
        tags = ['x', 'y', 'z']
        for e in range(self.dimension):
            error_coordinates[e] = sqrt(error_coordinates[e]/N)
            print(f"RMS Error for {tags[e]}-coordinate: {error_coordinates[e]}")
            print(f"Max error for {tags[e]}-coordinate (mm): {max_err_coordinates[e]}")
            print("-------------------------")
        return error


if __name__ == "__main__":
    from robot import FANUC_R_2000i
    robot = FANUC_R_2000i()
    d = robot.robot_configs.get_links_dimensions()
    calib = Calibration(robot=robot)
    # calib.calibrate(alpha=0.07)
    
    postfix = " (5)"#" (0)"
    
    T_base = np.load(f"T_base{postfix}.npy")
    # T_base = translation_x(0)
    
    T_tool = np.load(f"T_tool{postfix}.npy")
    # T_tool[0,:3,3] = np.zeros((3,))
    # T_tool[1,:3,3] = np.zeros((3,))
    # T_tool[2,:3,3] = np.zeros((3,))
    
    pi = np.load(f"pi{postfix}.npy")
    # pi = np.zeros((18, 1))
    # pi = np.array([d[1], 0,0,0,0,0,0,0, d[5], d[4], 0, 0,0,0,0,0,0,0], dtype=np.float).reshape((18, 1)) 

    np.set_printoptions(precision=3, suppress=True,)
    print(f"Pi:\n{np.array2string(pi, separator=', ')}")
    print(f"T_base:\n{np.array2string(T_base, separator=', ')}")
    print(f"T_tool:\n{np.array2string(T_tool, separator=', ')}")
    calib.RMS_report(pi=pi, T_base=T_base, T_tool=T_tool)
    # mat = calib.get_dataset_raw()
    # print(mat)
    # calib.visualize()
    
    # sample = calib.splitter()
    # print(sample[0] - mat["mA"][0], sample[1] - mat["mB"][0], sample[2] - mat["mC"][0])
    
    