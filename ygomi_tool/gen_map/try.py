import matplotlib.pyplot as plt
import numpy as np
import math
from scipy import spatial
dim = 2
size = 5
data = np.empty((size, dim))
for d in range(dim):
    data[:,d] = np.random.random(size)
tree = spatial.KDTree(data)
query = np.empty((1, 2))
query[0,0]=0.5
query[0,1]=0.5
knnIndex = tree.query_ball_point(query, 0.1)
print(knnIndex)



did
