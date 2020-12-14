import scipy.io
import numpy as np
import visualization as visual
import time

class Calibration:
    def __init__(self, dataset_file="calibration_dataset.mat",
                       num_configs=24, num_samples=10, num_positions=3,
                       positions_tags=["mA", "mB", "mC"]):
        self.dataset_file = dataset_file
        self.num_configs = num_configs
        self.num_samples = num_samples
        self.num_positions = num_positions
        self.positions_tags = positions_tags
        self.dimension = 3
        
        self.visualization_radius = {"link":0.003, "joint":0.004, "node":0.003, "axe":0.003, "trajectory_trail": 0.0009}
        self.visualization_scale = 0.1
        self.visualization_rate = 5

        
        self.read_mat_file()
        self.splitter()
        
    def get_dataset_raw(self):
        return self.dataset_raw
    
    def read_mat_file(self):
        self.dataset_raw = scipy.io.loadmat(self.dataset_file)
        
    def splitter(self):
        self.dataset = np.empty((self.num_configs, self.num_samples, self.num_positions, self.dimension))
        for i in range(self.num_configs):
            for j in range(self.num_samples):
                for k in range(self.num_positions):
                    self.dataset[i, j, k] = self.dataset_raw[self.positions_tags[k]][j+self.num_samples*i]
        # return(self.dataset[0,0])

    def get_std_config(self, config=None, config_idx=None):
        if(config_idx is not None):
            config = self.dataset[config_idx]
        if(config is None):
            raise AttributeError('Please provide parameter config or config_idx')
        std = []
        # print(config[:,0]) # 10 repetitions for mA in the configuration
        for i in range(self.num_positions):
            std.append(np.std(config[:,i], axis=0))
        return std
        
    def get_mean_config(self, config=None, config_idx=None):
        if(config_idx is not None):
            config = self.dataset[config_idx]
        if(config is None):
            raise AttributeError('Please provide parameter config or config_idx')
        mean = []
        for i in range(self.num_positions):
            mean.append(np.mean(config[:,i], axis=0))
        return mean      

    def visualize(self):
        vis = visual.RobotVisualization_vpython(rate=self.visualization_rate,
                                                scale=self.visualization_scale,
                                                radius=self.visualization_radius)
        frame = []
        for i, config in enumerate(self.dataset):
            print(f"{i+1}th Configuration")
        # for i in range(len(self.dataset)):
        # config = self.dataset[0]
            for sample in config:
                for pos in sample:       
                    frame.append(["node", pos])
                # print(f"Std for mA, mB, mC: {self.get_std_config(config_idx=0)}")
                # print(f"Mean: {self.get_mean_config(config_idx=0)}")
            print(f"Std for mA, mB, mC:\n{self.get_std_config(config=config)}")
            print(f"Mean:\n{self.get_mean_config(config=config)}")
            print("----------------------------------------------------------------")

        while True:
            vis.render_frame(frame, axis=True)
            # time.sleep(5)

            
if __name__ == "__main__":
    calib = Calibration()

    mat = calib.get_dataset_raw()
    calib.visualize()
    # sample = calib.splitter()
    # print(sample[0] - mat["mA"][0], sample[1] - mat["mB"][0], sample[2] - mat["mC"][0])
    
    print(len(mat["mA"]))
    