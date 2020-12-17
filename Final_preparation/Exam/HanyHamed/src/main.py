from EulerLagrangeDynamics2 import EulerLagrange2
from NewtonEulerDynamics import NewtonEuler
import numpy as np
from utils import *
from math import cos, sin,exp
from TrajectoryPlanning import TrajectoryPlanning
# Note: if you don't have tqdm install tqdm: pip3 install tqdm
# Note: The code for Euler-Lagrange is slow as I am using sympy and it takes time in substituting into the expressions

dyn_le = EulerLagrange2()
dyn_ne = NewtonEuler()

print("-----------------Task2-----------------")
(q1, q2) = ([0,0,0], [0.2,1,0.5])
f = 100
dq_max = [1, 0.1, 0.1]
ddq_max = [10, 5, 5]
traj_ptp, time = TrajectoryPlanning.PTP(q0=q1.copy(), qf=q2.copy(), f=f, dq_max=dq_max, ddq_max=ddq_max, debug=True)
print(f"Setpoints: Starting {q1}, Final {q2}")
print(f"Goal (Final) {q2}\nReal (Final): {traj_ptp[-1,:,0]}")
TrajectoryPlanning.plot_trajectory(traj=traj_ptp, title="PTP - Trapezoidal", time=time)

print("-----------------Task3-----------------")
print("--------------------- Polynomial 3rd order ---------------------")
(q0, qf) = ([0, 0, 0], [0.2, 1, 0.5])
t0,tf = 0, 2
(dq0, dqf) = ([0,0,0], [0,0,0])
traj_poly3 = TrajectoryPlanning.polynomial3(t0, q0, dq0, tf, qf, dqf)
TrajectoryPlanning.plot_trajectory(traj=traj_poly3, title="Polynomial - 3rd Order")

qt, dqt, ddqt = TrajectoryPlanning.extract_traj(traj_poly3)
ut = dyn_le.inverse(qt, dqt, ddqt)
plot_u(ut, n=3)


print("-----------------Task4-----------------")
print("--------------------- Polynomial 1st order ---------------------")
(u0, uf) = ([0, 0, -10], [4, 0, -6])
t0,tf = 0, 2
traj_poly1 = TrajectoryPlanning.polynomial1_tor(t0, u0, tf, uf)
ut = traj_poly1.squeeze()[:]
plot_u(ut, n=3)
(q0, dq0) = (np.array([0, 0, 0]), np.array([0, 0, 0]))
qt, dqt, ddqt = dyn_ne.direct(q0, dq0, ut, dt=1/1000)
plot_trajectory([qt, dqt, ddqt],n=3, dt=1/1000)

print("--------------------- LE ---------------------")
(q0, dq0) = (np.array([0, 0, 0]), np.array([0, 0, 0]))
dt= 1/1000
qt, dqt, ddqt = dyn_le.direct(q0, dq0, ut, dt=1/1000)
plot_trajectory([qt, dqt, ddqt],n=3, dt=dt)
