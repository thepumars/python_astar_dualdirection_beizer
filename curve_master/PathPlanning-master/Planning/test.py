import numpy as np
import random

# a = np.array([1, 2])
# b = [3, 4]
# c = {1, 4}
# d = {5, 6}
# e = {7, 3}
#
num_list = np.zeros(( 2, 5))
#
# for i in range(5):
#     for j in range(5):
#         point = [5+j*5, random.randint(5, 45)]
#         num_list[i, :, j] = point

traj = []
a = [9, 2, 8, 3, 10]
# b = [3, 4]
# c = [5, 6]
# c[1]
# traj.append(a)
# traj.append(b)
sorted_id = np.argsort(a)
print(np.sort(a))