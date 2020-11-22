import numpy as np
from robot import KUKA_KR10_R1100_2_configs

num_test_data = 20


# This function is only generating according to the limits of the joints without taking into consideration if it really feasiable or not
# For example if it zero configuration and the 2nd joint is turned that it will collide with the ground, here it is not taking into this consideration
def generate_random_angles(num):
    joints_limits = KUKA_KR10_R1100_2_configs.get_joints_limits()
    qss = []
    for i in range(num):
        qs = []
        for j in joints_limits:
            qs.append(np.random.uniform(j[0],j[1]))
        qss.append(qs)
    random_q  = qss
    return random_q

# special dataset (Hard-coded)
def generate_angels():
    qss_deg = [ [0 , 0, 0,    0, 0, 0],
                [45, 0, 0,    0, 0, 0],
                [0, -45, 0,    0, 0, 0],
                [45, -45, 0,    0, 0, 0],
                [0, -45, -45,    0, 0, 0],
                [0, 0, 0,    45, 0, 0],
                [0, 0, 0,    0, -45, 0],
                [0, 0, 0,    0, 0, 45],
                [0, 0, 0,    45, 0, 45],
                [10, 0, 0, 0, -45, 0]
         ]
    qss_rad = []
    for qs in qss_deg:
        qs_rad = []
        for j in qs:
            qs_rad.append(j*np.pi/180)
        qss_rad.append(qs)
    return qss_rad


def to_rad(deg):
    return deg*np.pi/180

# Generate some angles about a specific configuration
def generate_likelihood_angles(num):
    normal_orientation = [0, 0, 0,   0,0,0]
    joints_limits = KUKA_KR10_R1100_2_configs.get_joints_limits()
    offsets = [joints_limits[0],
               (-to_rad(10), to_rad(10)),
               (-5, 5),
               (-to_rad(10), to_rad(10)),
               (-to_rad(10), to_rad(10)),
               joints_limits[5]]
    qss = []
    for i in range(num):
        qs = []
        for idx,j in enumerate(offsets):
            qs.append(normal_orientation[idx]+np.random.uniform(j[0],j[1]))
        qss.append(qs)
    random_q  = qss
    return random_q


# test_data_FK = generate_angels()
# test_data_FK = generate_random_angles(10)
test_data_FK = generate_likelihood_angles(10)
print(test_data_FK)
np.save("test_data_FK.npy", np.array(test_data_FK))
