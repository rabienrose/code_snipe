import numpy as np
pose =np.matrix([[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])
x = np.matrix([[1],[1],[1]])
pose[0:3,1] = x
print(pose)