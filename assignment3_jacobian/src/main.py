from robot import KUKA_KR10_R1100_2
import numpy as np
from utils import calc_error

robot = KUKA_KR10_R1100_2()

def calc_jacobian(q):
    skew = robot.jacobian(q, method="skew")
    numerical = robot.jacobian(q, method="numerical")
    # print(np.linalg.det(numerical))
    # u, s, v = np.linalg.svd(numerical)
    # print(s)
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

# gen_data_FK = np.array()
# q = [0, 0, 0,   0,np.pi/4,0] # elbow
# q = [0, -np.pi/4, 0,   0,np.pi/8,0] # not singularity
# q = [0, -np.pi/4, 0,   0,0,0] # wrist
# q = [0.0, -1.9665760754252286, 0.8581859248230211, 0.0, 0.0, 0.0] # shoulder
q = [0.0, -np.pi/2, 0, 0.0, -(np.pi/2-np.arccos(25/90)), 0.0] # shoulder singularity
J = robot.jacobian(q, method="numerical")
u, s, v = np.linalg.svd(J)
print(s)
print(np.linalg.det(J))

# q = [0, 0, 0,   0,0,0]
# J = robot.jacobian(q, method="numerical")
# u, s, v = np.linalg.svd(J)
# print(s)

