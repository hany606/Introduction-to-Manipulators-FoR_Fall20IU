from robot import RRR_robot
import numpy as np
from utils import calc_error, get_position, get_rotation

robot = RRR_robot()

def calc_jacobian(q, debug=True):
    skew = robot.jacobian(q, method="skew")
    numerical = robot.jacobian(q, method="numerical")
    err = calc_error(skew, numerical)
    robot.check_singularity(q, debug=debug)
    if(debug == True):
        print(f"Error between computation of jacobian with skew theory and numerical derivatives method -> {err}")
        print("------------------------------------------------------")
    return err

q = [0.0, -1.5707963267948966, 1.5707963267948966]
# q = [0, 0, 0, 0, 0, 0]
T = robot.forward_kinematics(q, plot=True, debug=False)
q_calc = robot.inverse_kinematics(T, plot=False, debug=False)
T_calc = robot.forward_kinematics(q_calc, plot=False, debug=False)
# T_calc = robot.forward_kinematics(q_calc, plot=True, debug=False)
# print(f"Original q: {q}\nComputed q: {q_calc}")
# print("------------------------------------------------------")
# robot.print_frame(T)
# print("------------------------------------------------------")
# robot.print_frame(T_calc)
# print("------------------------------------------------------")
print(f"Error using FK then IK then FK -> {calc_error(T, T_calc)}")
print("#########################################################################")

### For testing single point
q = [0, -np.pi/2, np.pi/2]
calc_jacobian(q)

### For testing the test dataset
# gen_data_FK = np.load('test_data_FK.npy')

# total_err = 0
# for i,q_data in enumerate(gen_data_FK):
#     q = q_data
#     total_err += calc_jacobian(q, debug=False)

# print("######################################################")
# print(f"Error between computation of jacobian with skew theory and numerical derivatives method for {len(gen_data_FK)} configurations-> \n Average Error: {total_err/len(gen_data_FK)} \t Total Error: {total_err}")
# print("------------------------------------------------------")

### For Plotting all the test_data
# Ts = []
# for i in range(360):
#     q = [i*np.pi/180,0,(i-50)*np.pi/180]
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)

# robot.plot_robot_multi_frames(Ts, rate_factor=50)

Ts = []
trail = []
(q1, q2) = ([0,0,0], [-0.9,-2.3,1.2])
f = 10
dq_max = 1
ddq_max = 10
traj_ptp = robot.PTP(q1.copy(), q2.copy(), f, dq_max, ddq_max)
for traj in traj_ptp:
    q = traj[:,0]
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)
    trail.append(get_position(T[-1]))
robot.plot_robot_multi_frames(Ts, rate_factor=50, trail=trail)