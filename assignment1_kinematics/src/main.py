from robot import KUKA_KR10_R1100_2
import visualization as visual
import numpy as np
from utils import calc_error

vis = visual.RobotVisualization_vpython()
robot = KUKA_KR10_R1100_2()

### For plotting:
# q = [0, -np.pi/2, np.pi/2 + np.pi/4,   0,0,0]
# T = robot.forward_kinematics(q, plot=True, debug=True)

# To get a singularity
# T = np.zeros((4,4))
# T[:3,3] = np.array([0,0,-1000]).reshape((3,))
# q = robot.inverse_kinematics(T, plot=False, debug=False)
# print(q)
# T_calc = robot.forward_kinematics(q, plot=True, debug=False)

# q = [0.0, -np.pi/2, 0, 0.0, -(np.pi/2-np.arccos(25/90)), 0.0] # shoulder singularity
# q = [0.0, -np.pi/2, np.pi/6, 0.0, -(np.pi/2-np.arccos((25+(90+515+25)*np.cos(np.pi/3))/90)), 0.0] # shoulder singularity
# q = [0.0, -np.pi/2, np.pi/12,0,0, 0.0] # shoulder singularity
# print(25+np.cos(np.pi/2 - np.pi/12)*(25+90+515))
# T = robot.forward_kinematics(q, plot=True, debug=True)

### For testing single point
q = [0, -np.pi/2, np.pi/2,   0,0,0]
T = robot.forward_kinematics(q, plot=False, debug=False)
robot.print_frame(T)
q_calc = robot.inverse_kinematics(T, plot=False, debug=False)
T_calc = robot.forward_kinematics(q_calc, plot=False, debug=False)
print(f"Original q: {q}\nComputed q: {q_calc}")
print("------------------------------------------------------")
robot.print_frame(T)
print("------------------------------------------------------")
robot.print_frame(T_calc)
print("------------------------------------------------------")
print(f"Error using FK then IK then FK -> {calc_error(T, T_calc)}")
print("#########################################################################")

### For testing the test dataset
gen_data_FK = np.load('test_data_FK.npy')

for i,q_data in enumerate(gen_data_FK):
    q = q_data
    T = robot.forward_kinematics(q, plot=False, debug=False)
    robot.print_frame(T)
    q_calc = robot.inverse_kinematics(T, plot=False, debug=False)
    T_calc = robot.forward_kinematics(q_calc, plot=False, debug=False)
    print(f"Original q: {q}\nComputed q: {q_calc}")
    # print("------------------------------------------------------")
    # robot.print_frame(T)
    # print("------------------------------------------------------")
    # robot.print_frame(T_calc)
    # print("------------------------------------------------------")
    print(f"Error using FK then IK then FK (Test data #{i+1})-> {calc_error(T, T_calc)}")
    print("#########################################################################")

### For Plotting all the test_data
gen_data_FK = np.load('test_data_FK.npy')
Ts = []
for i,q_data in enumerate(gen_data_FK):
    q = q_data
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)

robot.plot_robot_multi_frames(Ts)