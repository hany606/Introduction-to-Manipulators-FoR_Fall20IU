import numpy as np
from IK import IK
from FK import FK

num_test_data = 20


# based on FK
def generate_random_positions():
    # random_q = ??? # generate random q with some constraints of the joints angels
    random_q = 0
    return random_q, FK(random_q)[-1]

test_data_FK = []
test_data_IK = []

for i in range(num_test_data):
    FK, IK = generate_random_positions()
    test_data_FK.append(FK)
    test_data_IK.append(IK)
    pass

np.save("test_data_FK.npy", np.array(test_data_FK))
np.save("test_data_IK.npy", np.array(test_data_IK))