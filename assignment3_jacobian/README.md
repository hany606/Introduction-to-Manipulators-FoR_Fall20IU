# Assignment3-Jacobian

In this directory, jacobian is being calculated by 2 methods Skew theory and Numerical Derivative based methods, and calculate the errors between the computed jacobians of the two methods. Moreover, the singularities are identifeid by observing the jacobian matrix in 3 ways: Rank, Determinant, and SVD.

## How to use?

1. Run main.py:
   ```bash
      python3 main.py
   ```
  The main file consists of different parts including the testing part by calculating the error and report the total and average error, and specifying multiple configurations with different singularty types.

## Note:

In order to run the code, you must have the dependencies (numpy) and in order to visualize, you must have (vpython)

## Some images from singularity configurations:

- Shoulder Singularity:

![Shoulder](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment3_jacobian/imgs/shoulder_singularity.gif)

- Schema for shoulder singularity:

![Shoulder](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment3_jacobian/imgs/shoulder_singularity.png)

- Schema for alignment singularity:

![alignment](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment3_jacobian/imgs/Alignment_singularity.png)



- Wrist Singularity:

![Wrist](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment3_jacobian/imgs/wrist%20singularity.png)

- Elbow Singularity:

![Elbow](https://github.com/hany606/FoR_Fall20IU/blob/main/assignment3_jacobian/imgs/elbow_singularity.png)
