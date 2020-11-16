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
gen_data_FK = np.load('test_data_FK.npy')

total_err = 0
for i,q_data in enumerate(gen_data_FK):
    q = q_data
    total_err += calc_jacobian(q, debug=False)

print("######################################################")
print(f"Error between computation of jacobian with skew theory and numerical derivatives method for {len(gen_data_FK)} configurations-> \n Average Error: {total_err/len(gen_data_FK)} \t Total Error: {total_err}")
print("------------------------------------------------------")


# -------------------------- Singularities --------------------------
# 1. Shoulder:
print("Shoulder singularity:")
q = [0, -1.6154540260077814, +0.04465769921288487, 0, np.pi/6, 0.0] # shoulder singularity (1st and 4th)
robot.check_singularity(q)
print("------------------------------------------------------")
# Ts = []
# for i in range(10):
#     q = [i*np.pi/180, -np.pi/6, -1.990468423823186, 0,0, 0.0] # shoulder singularity
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
# robot.plot_robot_multi_frames(Ts)

print("Shoulder (Alignment) singularity:")
q = [0, -np.pi/6, -2.2829525304720546, 0, np.pi/6, 0.0] # shoulder(Alignment) singularity (1st and 6th)
robot.check_singularity(q)
print("------------------------------------------------------")
# Ts = []
# for i in range(10):
#     q = [i*np.pi/180, -np.pi/6, -1.990468423823186, 0,0, 0.0] # shoulder singularity
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
# robot.plot_robot_multi_frames(Ts)

# 2. Wrist:
print("Wrist singularity:")
q = [0, -np.pi/4, np.pi/3,   10*np.pi/180,0, 20*np.pi/180] # wrist
robot.check_singularity(q)
print("------------------------------------------------------")
# T = robot.forward_kinematics(q, plot=True, debug=False)
# Ts = []
# for i in range(180):
#     q = [0, -np.pi/4, 0,   i*np.pi/180,0, (i+10)*np.pi/180] # wrist
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
# robot.plot_robot_multi_frames(Ts)

# 3. Elbow
print("Elbow singularity:")
q = [0, 0, 0,   0,-np.pi/2,0] # elbow
robot.check_singularity(q)
print("------------------------------------------------------")
# T = robot.forward_kinematics(q, plot=True, debug=False)
# Ts = []
# for i in range(180):
#     q = [0, 0, 0,   0,np.pi/4,0] # elbow
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
# robot.plot_robot_multi_frames(Ts)


print("Elbow and wrist singularity:")
q = [0, 0, 0,   0, 0, 0] # elbow & wrist
robot.check_singularity(q)
print("------------------------------------------------------")

# q = [0, -np.pi/8, -np.pi/8,   -np.pi/8,-np.pi/2,-np.pi/10] # not singularity
# q = [0, -np.pi/4, np.pi/3,   0,np.pi/8,0] # not singularity
# robot.check_singularity(q)
print("-------------------------------------------------------------------------")

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
    total_err += calc_error(T, T_calc)
    if(calc_error(T, T_calc) > 0.005):
        num_issues += 1
        print(f"Error in Configuration with index {i}\nOriginal q: {q}\nComputed q: {q_calc}")

    # print(f"Error using FK then IK then FK (Test data #{i+1})-> {calc_error(T, T_calc)}")
    # print("#########################################################################")
print("######################################################")
print(f"Number of configurations that has been calculated wrongly = {num_issues}")
print(f"Error using FK then IK then FK for {len(gen_data_FK)} configurations-> \n Average Error: {total_err/len(gen_data_FK)} \t Total Error: {total_err}")
print("------------------------------------------------------")
