# Assignment5-Dynamics

In this directory, Euler-Lagrange dynamic model has been implemented for RR plananr manipulator using moving frames algorithm (Direct and Inverse Dynamics) and Newton-Euler dynamic model for inverse problem using recursive algorithm.

## How to use?

1. Run main.py:
   ```bash
      python3 main.py
   ```
## Note:

In order to run the code, you must have the dependencies (numpy) and in order to visualize, you must have (vpython)

## Dynamics experiments:

Control inputs graphs in the following plots, they are the output of the inverse newton dynamics after being fed with the output of the direct lagrange dynamics. They are equivalent to the values that should be given to the joints in the other direction in order to stop the motion

* Configuration1 without any control

![Configuration 1](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config1_traj.png)

![Configuration 1](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/u_config1_ne.png)

![Configuration 1](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config1.gif)

* Configuration2 without any control

![Configuration 2](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config2_traj.png)

![Configuration 2](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/u_config2_ne.png)

![Configuration 2](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config2.gif)

* Configuration2 with control on the 1st joint as sin function

![Configuration 2](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config2_u_sin.gif)

* Configuration3 without any control

![Configuration 3](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config3_traj.png)

![Configuration 3](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/u_config3_ne.png)

![Configuration 3](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config3.png)

* Configuration3 without initial velocity

![Configuration 3](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config3_init_vel.gif)



* Configuration4 without any control

![Configuration 4](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config4_traj.png)

![Configuration 4](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/u_config4_ne.png)

![Configuration 4](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment5_dynamics/imgs/config4.gif)


## TODO:

* Make the function works in an online manner (adding step function in the class) instead of calculating everything in the function and leave the user to step the dynamics
* Refactor the code (Make it more OOP, more readable, more comments, better naming, combine the two classes in one file, add it to robot.py)
* Implement direct lagrange dynamics using jacobian instead
* Generalize the code to work with different types of manipulators easy
