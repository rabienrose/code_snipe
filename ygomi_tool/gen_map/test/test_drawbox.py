import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import slamfunction



def orthogonal_proj(zfront, zback):
    a = (zfront+zback)/(zfront-zback)
    b = -2*(zfront*zback)/(zfront-zback)
    return np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,a,b],
                        [0,0,0,zback]])

radius =10
box_count= 3
length = 4
center = [0,0,0]
N = 2
point_3ds =[]
points_box = slamfunction.draw_box(length,center,N)
point_3ds.extend(points_box)
for i in range(0, box_count):
    x = random.uniform(-radius-5, radius+5)
    z = random.uniform(-radius - 5, radius + 5)
    y = random.uniform(-5, 5)
    theta_x = 0
    theta_y = random.uniform(0, 2*3.1415926)
    theta_z = 0
    '''
    x = 10
    y = 0
    z = 0
    theta_x = 0
    theta_y = 30
    theta_z = 0
    '''
    pose_t = slamfunction.create_pose(x, y, z, theta_x, theta_y, theta_z)
    points = slamfunction.transform_point3d(points_box, pose_t)
    point_3ds.extend(points)

fig = plt.figure(figsize=(6, 6))
ax  = Axes3D(fig)
for i in range(0,len(point_3ds)):
   ax.scatter(point_3ds[i][0],point_3ds[i][1],point_3ds[i][2],c = 'r')
#proj3d.persp_transformation = orthogonal_proj
ax.set_zlabel('Z')
ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.view_init(0, -90)
slamfunction.axisEqual3D(ax)
plt.show()
