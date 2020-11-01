from IK import IK
from FK import FK
from visualization import RobotVisualization
import numpy as np

vis = RobotVisualization()

gen_data_FK = np.load('test_data_FK.npy')
gen_data_IK = np.load('test_data_IK.npy')

for i in range(len(gen_data_FK)):
    FK(gen_data_FK[i])

for i in range(len(gen_data_IK)):
    IK(gen_data_IK[i])
