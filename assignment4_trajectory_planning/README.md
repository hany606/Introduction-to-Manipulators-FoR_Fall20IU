# Assignment4-Trajectory-Planning

In this directory, analysis for FK, IK and Jacobian for RRR robot. Moreover, trajectory planning is implemented with 2 profiles (Polynomial with 5th order and Trapezoidal Velocity Profile) and 2 commands (PTP: Point to Point, LIN: Linear motion from point to another) and both of them with Trapezoidal Velocity Profile.

## How to use?

1. Run main.py:
   ```bash
      python3 main.py
   ```
## Note:

In order to run the code, you must have the dependencies (numpy) and in order to visualize, you must have (vpython)

## Plots for Trajectory Planning:

- 5th order Polynoimal with the following constraints:
```python
   (q0, qf) = ([-0.5, -0.6, 0], [1.57, 0.5, -2.0])
   t0,tf = 0, 2
   (dq0, dqf) = ([0,0,0], [0,0,0])
   (ddq0, ddqf) = ([0,0,0], [0,0,0])
```
![poly5-config1](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_polynomial_5th_order(Task2).png)

- 5th order Polynoimal with the following constraints:
```python
   (p1, p2) = ([1,0,2], [1/sqrt(2),1/sqrt(2),1.2])
   (q0, qf) = (robot.inverse_kinematics(p1), robot.inverse_kinematics(p2)
   t0,tf = 0, 2
   (dq0, dqf) = ([0,0,0], [0,0,0])
   (ddq0, ddqf) = ([0,0,0], [0,0,0])
```
![poly5-config1](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_polynomial_5th_order(united_config).png)