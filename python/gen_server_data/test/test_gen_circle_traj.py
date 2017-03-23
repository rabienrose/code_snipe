import slamfunction
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
def orthogonal_proj(zfront, zback):
    a = (zfront+zback)/(zfront-zback)
    b = -2*(zfront*zback)/(zfront-zback)
    return np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,a,b],
                        [0,0,0,zback]])
radius = 25
frame_count = 40
view_angle = 120
pose_list = slamfunction.gen_circle_traj(radius,frame_count)
fig = plt.figure(figsize=(6, 6))
ax = Axes3D(fig)
for pose in pose_list:
    slamfunction.show_camera(ax, pose, view_angle,'b')
ax.set_zlabel('Z')
ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.set_xlim(-30, 30)
ax.set_ylim(-30, 30)
ax.set_zlim(-30, 30)
ax.view_init(0, -90)
slamfunction.axisEqual3D(ax)
plt.show()