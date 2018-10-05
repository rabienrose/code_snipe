import slamfunction
import math
import numpy as np
import matplotlib.pyplot as plt

length = 10
center = [25,25,25]
N = 5

points_box = slamfunction.draw_box(length,center,N)
ids = []
width = 1024
height = 768
view_angle = 120
theta_x = 0
theta_y = 20
theta_z = 50
cen_x =0
cen_y=0
cen_z=0
view_angle_radian = view_angle/360*2*3.1415926
fx = width/2/math.tan(view_angle_radian/2)
fy = fx
cx=width/2
cy=height/2
'''
k= [[fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]]
'''
k = np.matrix([[fx,0,cx,0],[0,fy,cy,0],[0,0,1,0]])

pose = slamfunction.create_pose(cen_x, cen_y, cen_z,theta_x, theta_y, theta_z )
uvs,ids = slamfunction.project_3d(k, pose, points_box, width, height,ids)
fig = plt.figure()
ax = fig.gca()
#ax.plot(u, v,'k.')
plt.axis([0,1024,0,768])
for i in range(0,len(uvs)):
    ax.plot(uvs[i][0],uvs[i][1],'k.')
ax.set_aspect('equal')
plt.show()