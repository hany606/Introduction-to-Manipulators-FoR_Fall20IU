from robot import FANUC_R_2000i
from Calibration import Calibration
import numpy as np

robot = FANUC_R_2000i()
calib = Calibration(robot=robot)
# calib.calibrate(alpha=0.7)
postfix = " (5)"
pi = np.load(f"gen/pi{postfix}.npy")
T_base = np.load(f"gen/T_base{postfix}.npy")
T_tool = np.load(f"gen/T_tool{postfix}.npy")
np.set_printoptions(precision=3, suppress=True,)
print(f"Pi:\n{np.array2string(pi, separator=', ')}")
print(f"T_base:\n{np.array2string(T_base, separator=', ')}")
print(f"T_tool:\n{np.array2string(T_tool, separator=', ')}")
calib.RMS_report(pi=pi, T_base=T_base, T_tool=T_tool)
# mat = calib.get_dataset_raw()
# print(mat)
# calib.visualize()

# sample = calib.splitter()
# print(sample[0] - mat["mA"][0], sample[1] - mat["mB"][0], sample[2] - mat["mC"][0])

