from robot import KUKA_KR10_R1100_2
import numpy as np
from utils import calc_error, get_position, get_rotation

robot = KUKA_KR10_R1100_2()

def calc_jacobian(q, debug=True):
    skew = robot.jacobian(q, method="skew")
    numerical = robot.jacobian(q, method="numerical")
    err = calc_error(skew, numerical)
    robot.check_singularity(q, debug=debug)
    if(debug == True):
        print(f"Error between computation of jacobian with skew theory and numerical derivatives method -> {err}")
        print("------------------------------------------------------")
    return err
### For testing single point
# q = [0, -np.pi/2, np.pi/2,   0,0,0]
# calc_jacobian(q)

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
Ts = []
for i in range(360):
    q = [i*np.pi/180,0,(i-50)*np.pi/180]
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)

robot.plot_robot_multi_frames(Ts, rate_factor=50)