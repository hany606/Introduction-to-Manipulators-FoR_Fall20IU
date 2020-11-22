from robot import KUKA_KR10_R1100_2
import numpy as np
from utils import calc_error, get_position, get_rotation

robot = KUKA_KR10_R1100_2()
### For testing single point
# q = [0, -np.pi/2, np.pi/2,   0,0,0]
# calc_jacobian(q)

### For testing the test dataset
gen_data_FK = np.load('test_data_FK.npy')
gen_data_FK = gen_data_FK[:10]


print("Jacobian")
total_err = 0
for i,q_data in enumerate(gen_data_FK):
    q = q_data
    skew = robot.jacobian(q, method="skew")
    print(skew)


print("FK & IK Checking")

### For testing the test dataset
total_err = 0
sp_debug = [None, None]
num_issues = 0
for i,q_data in enumerate(gen_data_FK):
    q = q_data
    T = robot.forward_kinematics(q, plot=False, debug=False)
    # robot.print_frame(T)
    q_calc = robot.inverse_kinematics(T, plot=False, debug=False)
    T_calc = robot.forward_kinematics(q_calc, plot=False, debug=False)
    # if(i == 4):
    #     sp_debug[0] = q
    #     sp_debug[1] = q_calc
    # print(f"Original q: {q}\nComputed q: {q_calc}")
    # print("------------------------------------------------------")
    # robot.print_frame(T)
    # print("------------------------------------------------------")
    # robot.print_frame(T_calc)
    # print("------------------------------------------------------")
    err = calc_error(T, T_calc)
    total_err += err
    if(err > 0.005):
        num_issues += 1
        print(f"Error {err} in Configuration with index {i}\nOriginal q: {q}\nComputed q: {q_calc}")

    # print(f"Error using FK then IK then FK (Test data #{i+1})-> {calc_error(T, T_calc)}")
    # print("#########################################################################")
print("######################################################")
# print(f"Number of configurations that has been calculated wrongly = {num_issues}")
# print(f"Error using FK then IK then FK for {len(gen_data_FK)} configurations-> \n Average Error: {total_err/len(gen_data_FK)} \t Total Error: {total_err}")
print("------------------------------------------------------")