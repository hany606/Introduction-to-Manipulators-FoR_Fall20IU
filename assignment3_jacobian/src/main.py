from robot import KUKA_KR10_R1100_2
import numpy as np
from utils import calc_error

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
q = [0, -np.pi/6, -1.990468423823186,0,0, 0.0] # shoulder singularity
robot.check_singularity(q)
print("------------------------------------------------------")
# Ts = []
# for i in range(360):
#     q = [i*np.pi/180, -np.pi/6, -1.990468423823186,0,0, 0.0] # shoulder singularity
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
# robot.plot_robot_multi_frames(Ts)

# 2. Wrist:
print("Wrist singularity:")
q = [0, -np.pi/4, 0,   10*np.pi/180,0, 20*np.pi/180] # wrist
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
