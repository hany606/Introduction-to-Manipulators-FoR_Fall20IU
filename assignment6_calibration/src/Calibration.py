import scipy.io
import numpy as np
import visualization as visual
import time
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
                       reference_points_tags=["mA", "mB", "mC"]):
        self.dataset_file = dataset_file
        self.num_configs = num_configs
        self.num_samples = num_samples
        self.num_reference_points = num_reference_points
        self.reference_points_tags = reference_points_tags
        self.num_joints = robot.num_joints
        self.dimension = 3
        self.num_unknown_parameters = 4+8+6 # Only the robot # 27: including T_base and T_tool
        self.robot = robot
        
        self.jacobian = Jacobian(self.robot)
        
        self.pi_0 = np.zeros((self.num_unknown_parameters, 1))#? # TODO
        
        self.d = robot.robot_configs.get_links_dimensions()

        
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
        return [vec[0]*1000, vec[1]*1000, vec[2]*1000,]
    def splitter(self):
        self.configruations = np.empty((self.num_configs, self.num_samples, self.num_joints, 1))
        self.dataset = np.empty((self.num_configs, self.num_samples, self.num_reference_points, self.dimension))
        for i in range(self.num_configs):
            for j in range(self.num_samples):
                self.configruations[i, j] = np.array(self.dataset_raw["q"][j+self.num_samples*i]).reshape((self.num_joints, 1))
                for k in range(self.num_reference_points):
                    self.dataset[i, j, k] = self._rescale(self.dataset_raw[self.reference_points_tags[k]][j+self.num_samples*i])
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
                # Calculate identification Jacobian J_pi
                for k in range(self.num_reference_points):
                    jacobian_pi = self.jacobian.calc_identification_jacobian(T_base=self.T_base, T_tool=self.T_tool[k], q=q, pi=self.pi, pi_0=self.pi_0)
                    jacobian_pi_jp = jacobian_pi[:3].reshape((3, self.num_unknown_parameters))
                    delta_p_i_j = (self.dataset[i,j,k].copy() - get_position(self.T_robot).copy()).reshape((3,1))
                    term = jacobian_pi_jp.dot(delta_pi) - delta_p_i_j
                    stopping_sum += term.T @ term
        print(stopping_sum)
        if(stopping_sum[0][0] < epsilon):
            return True
        return False
    
    def calibrate(self, max_num_steps=1000, alpha=0.001, epsilon=1e-8):
        steps = 0
        # define pi
        self.pi = self.pi_0
        while True:
            print(f"{steps+1}th iteration:")
            self.T_base, self.T_tool = self._step1()
            delta_pi = self._step2()
            self.pi += alpha*delta_pi
            
            print("Pi:")            
            print(self.pi)
            steps += 1
            if(steps >= max_num_steps or self._terminamtion_criteria(delta_pi, epsilon)):
                break
        print("-------------------------")
        print(self.pi)

if __name__ == "__main__":
    from robot import FANUC_R_2000i
    robot = FANUC_R_2000i()
    calib = Calibration(robot=robot)
    calib.calibrate(alpha=0.05)
    # mat = calib.get_dataset_raw()
    # print(mat)
    # calib.visualize()
    
    # sample = calib.splitter()
    # print(sample[0] - mat["mA"][0], sample[1] - mat["mB"][0], sample[2] - mat["mC"][0])
    
    