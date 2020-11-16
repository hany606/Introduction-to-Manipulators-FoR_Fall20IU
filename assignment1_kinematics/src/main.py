from robot import KUKA_KR10_R1100_2
import visualization as visual
import numpy as np
from utils import calc_error, get_position

vis = visual.RobotVisualization_vpython()
robot = KUKA_KR10_R1100_2()

### For plotting:
# q = [0, -np.pi/2, np.pi/2 + np.pi/4,   0,0,0]
# T = robot.forward_kinematics(q, plot=True, debug=True)

### For testing single point
q = [0, -np.pi/2, np.pi/2,   0,0,0]
# q = [0, 0, 0, 0, 0, 0]
T = robot.forward_kinematics(q, plot=False, debug=False)
robot.print_frame(T)
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

### For testing the test dataset
gen_data_FK = np.load('test_data_FK.npy')
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
    total_err += calc_error(T, T_calc)
    if(calc_error(T, T_calc) > 0.005):
        num_issues += 1
        print(f"Error in Configuration with index {i}\nOriginal q: {q}\nComputed q: {q_calc}")

    # print(f"Error using FK then IK then FK (Test data #{i+1})-> {calc_error(T, T_calc)}")
    # print("#########################################################################")
print("######################################################")
print(f"Number of configurations that calculated wrongly = {num_issues}")
print(f"Error using FK then IK then FK for {len(gen_data_FK)} configurations-> \n Average Error: {total_err/len(gen_data_FK)} \t Total Error: {total_err}")
print("------------------------------------------------------")

# Bugs:
### For Plotting all the test_data
# TODO: Problem in q[1] 
# Original q: [-1.98442436 -2.4481053   1.73708161 -1.0205817   2.09175942  5.24194555]
# Computed q: [-1.984424363238259, 0.693487349441859, 1.7370816140201808, -2.1210109560143975, 1.0498332372010095, 2.10035289888095]
# Original q: [-0.46427135 -2.96204417  2.71605409  0.36564022 -1.21925581  5.12524501]
# Computed q: [2.6773213005280083, 0.19852281370514469, 2.7169575741600283, 0.3438476668695268, 1.6629733934263387, 2.1476781706293893]
# Original q: [-1.12184022 -2.88786569  2.69169024 -2.71125964 -1.10978837 -1.20285425]
# Computed q: [-1.1218402234658011, 0.2537269663962973, 2.6916902421593356, 2.7112596365294195, 2.0318042845984228, -1.2028542484955125]
# Ts = []
# for i,q_data in enumerate(sp_debug):
#     q = q_data
#     T = robot.forward_kinematics(q, plot=False, debug=True, return_all=True)
#     Ts.append(T)

# robot.plot_robot_multi_frames(Ts)

### For Plotting all the test_data
gen_data_FK = np.load('test_data_FK.npy')
Ts = []
for i,q_data in enumerate(gen_data_FK):
    q = q_data
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)

robot.plot_robot_multi_frames(Ts)