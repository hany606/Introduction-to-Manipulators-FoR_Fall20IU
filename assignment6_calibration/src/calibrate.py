from robot import FANUC_R_2000i
from Calibration import Calibration
import numpy as np

robot = FANUC_R_2000i()
calib = Calibration(robot=robot)
calib.calibrate(alpha=0.05)
suffix = ""
pi = np.load(f"pi{suffix}.npy")
T_base = np.load(f"T_base{suffix}.npy")
T_tool = np.load(f"T_tool{suffix}.npy")
np.set_printoptions(precision=3, suppress=True,)
print(f"Pi:\n{np.array2string(pi, separator=', ')}")
print(f"T_base:\n{np.array2string(T_base, separator=', ')}")
print(f"T_tool:\n{np.array2string(T_tool, separator=', ')}")
calib.RMS_report(pi=pi, T_base=T_base, T_tool=T_tool)