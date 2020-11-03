from robot import KUKA_KR10_R1100_2
from visualization import RobotVisualization
import numpy as np
from utils import calc_error

vis = RobotVisualization()
robot = KUKA_KR10_R1100_2()


### For testing single point
# q = [0, -np.pi/2, np.pi/2,   0,0,0]
# T = robot.forward_kinematics(q, debug=False)
# robot.print_frame(T)
# q_calc = robot.inverse_kinematics(T, debug=False)
# T_calc = robot.forward_kinematics(q_calc, debug=False)
# print(f"Original q: {q}\nComputed q: {q_calc}")
# print("------------------------------------------------------")
# robot.print_frame(T)
# print("------------------------------------------------------")
# robot.print_frame(T_calc)
# print("------------------------------------------------------")
# print(f"Error using FK then IK then FK -> {calc_error(T, T_calc)}")
# print("#########################################################################")

### For testing the test dataset
gen_data_FK = np.load('test_data_FK.npy')

for i,q_data in enumerate(gen_data_FK):
    q = q_data
    T = robot.forward_kinematics(q, debug=False)
    # robot.print_frame(T)
    q_calc = robot.inverse_kinematics(T, plot=False, debug=False)
    T_calc = robot.forward_kinematics(q_calc, plot=False, debug=False)
    # print(f"Original q: {q}\nComputed q: {q_calc}")
    # print("------------------------------------------------------")
    # robot.print_frame(T)
    # print("------------------------------------------------------")
    # robot.print_frame(T_calc)
    # print("------------------------------------------------------")
    print(f"Error using FK then IK then FK (Test data #{i+1})-> {calc_error(T, T_calc)}")
    print("#########################################################################")