# Assignment4-Trajectory-Planning

In this directory, analysis for FK, IK and Jacobian for RRR robot. Moreover, trajectory planning is implemented with 2 profiles (Polynomial with 5th order and Trapezoidal Velocity Profile) and 2 commands (PTP: Point to Point, LIN: Linear motion from point to another) and both of them with Trapezoidal Velocity Profile.

## How to use?

1. Run main.py:
   ```bash
      python3 main.py
   ```
## Note:

In order to run the code, you must have the dependencies (numpy) and in order to visualize, you must have (vpython)

## Trajectory Planning profiles and commands:

- 5th order Polynoimal with the following constraints (As it is required from the assignment - Task 2):
```python
   (q0, qf) = ([-0.5, -0.6, 0], [1.57, 0.5, -2.0])
   t0,tf = 0, 2
   (dq0, dqf) = ([0,0,0], [0,0,0])
   (ddq0, ddqf) = ([0,0,0], [0,0,0])
```
![poly5-config-assignment Plot](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_polynomial_5th_order(Task2).png)

![poly5-config-assignment gif](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/poly5.gif)

- PTP command with Trapezoidal Profile with the following constraints (As it is required from the assignment - Task 3):
```python
   (q1, q2) = ([0,0,0], [-0.9,-2.3,1.2])
   f = 10
   dq_max = [1, 1, 1]
   ddq_max = [10, 10, 10]
```
![PTP-config-assignment plot](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_PTP_Trapezoidal(Task3).png)

![PTP-config-assignment gif](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/PTP.gif)

- LIN command with Trapezoidal Profile with the following constraints (As it is required from the assignment - Task 4):
```python
   (p1, p2) = ([1,0,2], [1/np.sqrt(2),1/np.sqrt(2),1.2])
   f = 10
   dp_max = [1, 1, 1]
   ddp_max = [10, 10, 10]
   num_samples = 5
```
![LIN-config-assignment plot cartesian](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_LIN_Trapezoidal_v2_cartesian_space(Task4).png)

![LIN-config-assignment plot joint](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_LIN_Trapezoidal_v2_joint_space(Task4).png)

![LIN-config-assignment gif](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/LIN.gif)


However, another version of LIN has been implemented before the currently used one that was shown in the previous plots. Using another method, it is described in the report. It uses sample of point in the space and then using IK to get the joints positions and use plan for joints positions, however, it shows not exactly trapezoidal porfile for the cartesian space planning.

* Using 2 points for sampling

![LIN-config-assignment plot v1 sample2](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_LIN_Trapezoidal_v1_sample2(Task4).png)

![LIN-config-assignment gif](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/LIN_v1_sample2.gif)

* Using 5 points for sampling

![LIN-config-assignment plot v1 sample5](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_LIN_Trapezoidal_v1_sample5(Task4).png)

* Using 50 points for sampling

![LIN-config-assignment plot v1 sample50](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_LIN_Trapezoidal_v1_sample50(Task4).png)

* Using 100 points for sampling

![LIN-config-assignment plot v1 sample100](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_LIN_Trapezoidal_v1_sample100(Task4).png)

![LIN-config-assignment gif](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/LIN_v1.gif)

![All-config-assignment gif](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/all.gif)



## Trajectory Planning profiles and commands:

The following initial and goal configuration in cartesian space has been used to test all the commands and profiles:

```python
   (p1, p2) = ([1,0,2], [1/sqrt(2),1/sqrt(2),1.2])
   (q1, q2) = (robot.inverse_kinematics(p1), robot.inverse_kinematics(p2)
```
![All-config-assignment gif](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/all_one_config.gif)


- 5th order Polynoimal:
```python
   (p1, p2) = ([1,0,2], [1/sqrt(2),1/sqrt(2),1.2])
   (q0, qf) = (robot.inverse_kinematics(p1), robot.inverse_kinematics(p2)
   t0,tf = 0, 2
   (dq0, dqf) = ([0,0,0], [0,0,0])
   (ddq0, ddqf) = ([0,0,0], [0,0,0])
```
![poly5-config-all](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_polynomial_5th_order(united_config).png)

- PTP command with Trapezoidal Profile with the following constraints:
```python
   (p1, p2) = ([1,0,2], [1/sqrt(2),1/sqrt(2),1.2])
   (q1, q2) = (robot.inverse_kinematics(p1), robot.inverse_kinematics(p2)
   f = 10
   dq_max = [1, 1, 1]
   ddq_max = [10, 10, 10]
```
![PTP-config-assignment plot](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_PTP_Trapezoidal(united_config).png)

- LIN command with Trapezoidal Profile with the following constraints:
```python
   (p1, p2) = ([1,0,2], [1/sqrt(2),1/sqrt(2),1.2])
   (q1, q2) = (robot.inverse_kinematics(p1), robot.inverse_kinematics(p2)
   f = 10
   dq_max = [1, 1, 1]
   ddq_max = [10, 10, 10]
```
![LIN-config-assignment plot cartesian](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_LIN_Trapezoidal_v2_cartesian_space(united_config).png)

![LIN-config-assignment plot joint](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment4_trajectory_planning/imgs/Trajectory_planning_LIN_Trapezoidal_v2_joint_space(united_config).png)



