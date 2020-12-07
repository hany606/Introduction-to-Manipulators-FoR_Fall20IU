from EulerLagrangeDynamics import EulerLagrange
from NewtonEulerDynamics import NewtonEuler
import numpy as np

EL_dyn = EulerLagrange()
NE_dyn = NewtonEuler()

q0 = np.array([-np.pi/2, np.pi/2]).reshape(2,1)
dq0 = np.array([0,0]).reshape(2,1)
ut = [np.array([0,0]).reshape(2,1) for i in range(1000)]
qt, dqt, ddqt = EL_dyn.direct(q0, dq0, ut)

ut_inverse = NE_dyn.inverse(qt, dqt, ddqt)
print(ut[0])
print(ut_inverse[0])

print(sum(np.array(ut[1:-1]).squeeze()-np.array(ut_inverse).squeeze()))

# Visualization

# Plots