from robot import KUKA_KR10_R1100_2
import numpy as np
from utils import calc_error

robot = KUKA_KR10_R1100_2()

def calc_jacobian(q):
    skew = robot.jacobian(q, method="skew")
    numerical = robot.jacobian(q, method="numerical")
    print(np.linalg.matrix_rank(numerical))

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

# -------------------------- Singularities --------------------------
# 1. Shoulder:
# q = [0, -np.pi/6, -1.990468423823186,0,0, 0.0] # shoulder singularity
# Ts = []
# for i in range(360):
#     q = [i*np.pi/180, -np.pi/6, -1.990468423823186,0,0, 0.0] # shoulder singularity
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
# robot.plot_robot_multi_frames(Ts)

# 2. Wrist:
# q = [0, -np.pi/4, 0,   10*np.pi/180,0, 20*np.pi/180] # wrist
# T = robot.forward_kinematics(q, plot=True, debug=False)
# Ts = []
# for i in range(180):
#     q = [0, -np.pi/4, 0,   i*np.pi/180,0, (i+10)*np.pi/180] # wrist
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
# robot.plot_robot_multi_frames(Ts)

# 3. Elbow
# q = [0, -np.pi/4, 0,   10*np.pi/180,0, 20*np.pi/180] # wrist
# T = robot.forward_kinematics(q, plot=True, debug=False)
# Ts = []
# for i in range(180):
#     q = [0, 0, 0,   0,np.pi/4,0] # elbow
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
# robot.plot_robot_multi_frames(Ts)


# gen_data_FK = np.array()
# q = [0, 0, 0,   0,0,0]
# q = [0, -np.pi/6, -1.990468423823186,0,0, 0.0] # shoulder singularity
# q = [0, -np.pi/4, 0,   10*np.pi/180,0, 20*np.pi/180] # wrist
# q = [0, -np.pi/4, 0,   0,0,0] # wrist
# q = [0, 0, 0,   -np.pi/8,-np.pi/2,-np.pi/10] # elbow singularity
# q = [0, -np.pi/8, -np.pi/8,   -np.pi/8,-np.pi/2,-np.pi/10] # not singularity
# q = [0, -np.pi/4, np.pi/3,   0,np.pi/8,0] # not singularity

J = robot.jacobian(q, method="numerical")
u, s, v = np.linalg.svd(J)
print(min(s),s)
print(np.linalg.det(J))
print(np.linalg.matrix_rank(J))
robot.forward_kinematics(q, debug=False)
