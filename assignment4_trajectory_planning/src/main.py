from robot import RRR_robot
import numpy as np
from utils import calc_error, get_position, get_rotation, pos2hom

robot = RRR_robot()
# ---------------------- FK & IK & Jacobian
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
T = robot.forward_kinematics(q, plot=False, debug=False)
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

### For Plotting FK
# Ts = []
# for i in range(360):
#     q = [i*np.pi/180,0,(i-50)*np.pi/180]
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
# robot.plot_robot_multi_frames(Ts, rate_factor=50)

print("--------------------- Required configurations in the assignment ---------------------------------")
print("--------------------- Polynomial 5th ordedr ---------------------")
(q0, qf) = ([-0.5, -0.6, 0], [1.57, 0.5, -2.0])
t0,tf = 0, 2
(dq0, dqf) = ([0,0,0], [0,0,0])
(ddq0, ddqf) = ([0,0,0], [0,0,0])
traj_poly5 = robot.trajectory_planning.polynomial5(t0, q0, dq0, ddq0, tf, qf, dqf, ddqf)
robot.trajectory_planning.plot_trajectory(traj=traj_poly5, title="Polynomial - 5th Order")

# Ts = []
# trail = []
# for traj in traj_poly5[:,:,0]:
#     q = traj
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
#     trail.append(get_position(T[-1]))
#     # print(trail[-1])
# robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)


print("--------------------- PTP ---------------------")
(q1, q2) = ([0,0,0], [-0.9,-2.3,1.2])
f = 10
dq_max = [1, 1, 1]
ddq_max = [10, 10, 10]
traj_ptp, time = robot.trajectory_planning.PTP(q1.copy(), q2.copy(), f, dq_max, ddq_max, debug=True)
print(f"Setpoints: Starting {q1}, Final {q2}")
print(f"Goal (Final) {q2}\nReal (Final): {traj_ptp[-1,:,0]}")
robot.trajectory_planning.plot_trajectory(traj=traj_ptp, title="PTP - Trapezoidal", time=time)


# Ts = []
# trail = []
# for traj in traj_ptp[:,:,0]:
#     q = traj
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
#     trail.append(get_position(T[-1]))
#     # print(trail[-1])
# robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)


print("--------------------- LIN ---------------------")
(p1, p2) = ([1,0,2], [1/np.sqrt(2),1/np.sqrt(2),1.2])
f = 10
dp_max = [1, 1, 1]
ddp_max = [10, 10, 10]
num_samples = 5
traj_lin, time, joint_traj = robot.trajectory_planning.LIN(robot, p1.copy(), p2.copy(), f, dp_max, ddp_max)
print(f"Setpoints: Starting {p1}, Final {p2}")
print(f"Goal (Final) {p2}\nReal (Final): {traj_lin[-1,:,0]}")
robot.trajectory_planning.plot_trajectory_cartesian(traj=traj_lin, title="2nd version - LIN - Trapezoidal - Cartesian Space", time=time)
robot.trajectory_planning.plot_trajectory(traj=joint_traj, title="2nd version LIN - Trapezoidal - Joint Space", time=time)

# Ts = []
# trail = []
# for traj in joint_traj[:,:,0]:
#     q = traj
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
#     trail.append(get_position(T[-1]))
#     # print(trail[-1])
# robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)


# All
Ts = []
trail = []
for traj in traj_poly5[:,:,0]:
    q = traj
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)
    trail.append(get_position(T[-1]))
for traj in traj_ptp[:,:,0]:
    q = traj
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)
    trail.append(get_position(T[-1]))
for traj in joint_traj[:,:,0]:
    q = traj
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)
    trail.append(get_position(T[-1]))
robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/10, trail=trail)


print("--------------------- Required configurations for the same point for all the profiles ---------------------------------")
(p1, p2) = ([1,0,2], [1/np.sqrt(2),1/np.sqrt(2),1.2])
(q1, q2) = (robot.inverse_kinematics(pos2hom(np.array(p1)), plot=False, debug=False), robot.inverse_kinematics(pos2hom(np.array(p2)), plot=False, debug=False))
print("--------------------- Polynomial 5th ordedr ---------------------")
(q0, qf) = ([-0.5, -0.6, 0], [1.57, 0.5, -2.0])
q0, qf = q1, q2
t0,tf = 0, 2
(dq0, dqf) = ([0,0,0], [0,0,0])
(ddq0, ddqf) = ([0,0,0], [0,0,0])
traj_poly5 = robot.trajectory_planning.polynomial5(t0, q0, dq0, ddq0, tf, qf, dqf, ddqf)
robot.trajectory_planning.plot_trajectory(traj=traj_poly5, title="Polynomial - 5th Order")

# Ts = []
# trail = []
# for traj in traj_poly5[:,:,0]:
#     q = traj
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
#     trail.append(get_position(T[-1]))
# robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)


print("--------------------- PTP ---------------------")
# (q1, q2) = ([0,0,0], [-0.9,-2.3,1.2])
f = 10
dq_max = [1, 1, 1]
ddq_max = [10, 10, 10]
traj_ptp, time = robot.trajectory_planning.PTP(q1.copy(), q2.copy(), f, dq_max, ddq_max, debug=True)
print(f"Setpoints: Starting {q1}, Final {q2}")
print(f"Goal (Final) {q2}\nReal (Final): {traj_ptp[-1,:,0]}")
robot.trajectory_planning.plot_trajectory(traj=traj_ptp, title="PTP - Trapezoidal", time=time)


# Ts = []
# trail = []
# for traj in traj_ptp[:,:,0]:
#     q = traj
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
#     trail.append(get_position(T[-1]))
#     # print(trail[-1])
# robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)


print("--------------------- LIN ---------------------")
(p1, p2) = ([1,0,2], [1/np.sqrt(2),1/np.sqrt(2),1.2])
f = 10
dp_max = [1, 1, 1]
ddp_max = [10, 10, 10]
num_samples = 5
traj_lin, time, joint_traj = robot.trajectory_planning.LIN(robot, p1.copy(), p2.copy(), f, dp_max, ddp_max)
print(f"Setpoints: Starting {p1}, Final {p2}")
print(f"Goal (Final) {p2}\nReal (Final): {traj_lin[-1,:,0]}")
robot.trajectory_planning.plot_trajectory_cartesian(traj=traj_lin, title="2nd version - LIN - Trapezoidal - Cartesian Space", time=time)
robot.trajectory_planning.plot_trajectory(traj=joint_traj, title="2nd version LIN - Trapezoidal - Joint Space", time=time)

# Ts = []
# trail = []
# for traj in joint_traj[:,:,0]:
#     q = traj
#     T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
#     Ts.append(T)
#     trail.append(get_position(T[-1]))
#     # print(trail[-1])
# robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/5, trail=trail)


# All
Ts = []
trail = []
for traj in traj_poly5[:,:,0]:
    q = traj
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)
    trail.append(get_position(T[-1]))
for traj in traj_ptp[:,:,0]:
    q = traj
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)
    trail.append(get_position(T[-1]))
for traj in joint_traj[:,:,0]:
    q = traj
    T = robot.forward_kinematics(q, plot=False, debug=False, return_all=True)
    Ts.append(T)
    trail.append(get_position(T[-1]))
robot.plot_robot_multi_frames(Ts, rate_factor=len(Ts)/20, trail=trail)

