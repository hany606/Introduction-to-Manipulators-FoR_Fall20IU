from robot import KUKA_KR10_R1100_2
import numpy as np
from utils import calc_error

robot = KUKA_KR10_R1100_2()

def calc_jacobian(q):
    skew = robot.jacobian(q, method="skew")
    numerical = robot.jacobian(q, method="numerical")
    print(f"Error between computation of jacobian with skew theory and numerical derivatives method -> {calc_error(skew, numerical)}")
    print("------------------------------------------------------")
### For testing single point
q = [0, -np.pi/2, np.pi/2,   0,0,0]
calc_jacobian(q)

### For testing the test dataset
gen_data_FK = np.load('test_data_FK.npy')

for i,q_data in enumerate(gen_data_FK):
    q = q_data
    calc_jacobian(q)
