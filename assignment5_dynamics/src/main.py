from EulerLagrangeDynamics import EulerLagrange
from NewtonEulerDynamics import NewtonEuler
import numpy as np
from utils import *
from visualization import RobotVisualization_vpython
from math import cos, sin,exp



EL_dyn = EulerLagrange()
NE_dyn = NewtonEuler()
dt = 0.0004
simulation_times_steps = 25000#5000#25000


# q0 = np.array([-np.pi/2, np.pi/2]).reshape(2,1) # config1
q0 = np.array([-np.pi/2, 0]).reshape(2,1) # no motion config2
# q0 = np.array([np.pi/2, 0]).reshape(2,1)    # no motion config3
# q0 = np.array([np.pi/2, np.pi/4]).reshape(2,1)  # config4
# q0 = np.array([0, 0]).reshape(2,1) #config5
# print(q0)
dq0 = np.array([0,0]).reshape(2,1)
# dq0 = np.array([0.5,0]).reshape(2,1)
# ut = [np.array([0,0]).reshape(2,1) for i in range(simulation_times_steps)]
ut = [np.array([sin(i/500),0]).reshape(2,1) for i in range(simulation_times_steps)]
plot_u(ut, dt=dt)

print(f"Computing the direct dynamics for simulation time: {simulation_times_steps*dt} seconds")
qt, dqt, ddqt = EL_dyn.direct(q0, dq0, ut, dt=dt, debug=False)

plot_trajectory([qt, dqt, ddqt], dt=dt)
# vis = RobotVisualization_vpython(rate=1000, scale=0.07, radius={"link":0.003, "joint":0.004, "node":0.004, "axe":0.003, "trajectory_trail": 0.0009})
# while True:
#     for i in range(len(qt)):
#         q = qt[i].copy()
#         frame = [
#                 ['link', [0, 0, 0], [0.8*cos(q[0]), 0.8*sin(q[0]), 0.]], 
#                 # ['link', [0., 0., 0.], [ 25.,   0., 400.]],
#                 ['link', [0.8*cos(q[0]), 0.8*sin(q[0]), 0.], [0.8*cos(q[0]) + 0.8*cos(q[0]+q[1]), 0.8*sin(q[0]) + 0.8*sin(q[0]+q[1]), 0.]],
#                 ['joint', [0, 0, 0]], 
#                 ['joint', [0.8*cos(q[0]), 0.8*sin(q[0]), 0.]]]
#         vis.render_frame(frame, axis=False)


ut_ne_inverse = NE_dyn.inverse(qt, dqt, ddqt)
ut_ne_inverse = np.array(ut_ne_inverse).reshape(len(ut_ne_inverse),2,1)
ut_el_inverse = np.array(EL_dyn.inverse(qt, dqt, ddqt)).astype(np.float64)
qt2, dqt2, ddqt2 = EL_dyn.direct(q0, dq0, ut_ne_inverse, dt=dt, debug=False)

plot_u(ut_ne_inverse, dt=dt)
vis = RobotVisualization_vpython(rate=1000, scale=0.07, radius={"link":0.003, "joint":0.004, "node":0.004, "axe":0.003, "trajectory_trail": 0.0009})
while True:
    for i in range(len(qt2)):
        q = qt2[i].copy()
        frame = [
                ['link', [0, 0, 0], [0.8*cos(q[0]), 0.8*sin(q[0]), 0.]], 
                # ['link', [0., 0., 0.], [ 25.,   0., 400.]],
                ['link', [0.8*cos(q[0]), 0.8*sin(q[0]), 0.], [0.8*cos(q[0]) + 0.8*cos(q[0]+q[1]), 0.8*sin(q[0]) + 0.8*sin(q[0]+q[1]), 0.]],
                ['joint', [0, 0, 0]], 
                ['joint', [0.8*cos(q[0]), 0.8*sin(q[0]), 0.]]]
        vis.render_frame(frame, axis=False)

        # while True:
            # vis.render_frame(frame, axis=False)


