import roadgeofunction
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

core_data={}
core_data['lines'] = []
core_data['lanes'] = []
points_list = [[-50,0,50],[-35,0,50],[-20,0,50],[-10,0,50],[0,0,50],[10,0,49],[20,0,45.8],[30,0,40],[40,0,30],[45,0,21.7],
               [50,0,0],[50,0,-10],[50,0,-20],[50,0,-30],[50,0,-50]]
points_new = roadgeofunction.interpolate3d(points_list,31)

# lines = core_data['lines']
half_N = 2
s = 8
# roadgeofunction.generateLine(lines,points_new,half_N,s)
# lanes = core_data['lanes']
# roadgeofunction.generateLane(lanes,points_new,half_N,s/2)
roadgeofunction.generate_road(core_data,points_new,half_N,s)
roadgeofunction.generate_veh_traj(core_data,0,2)





fig = plt.figure('fig', figsize=(6, 6))
ax = Axes3D(fig)
ax.set_axis_bgcolor('k')       #设置画布颜色
ax.view_init(0, 90)
# for i in range(0,len(points_list)):
#     ax.scatter3D(points_list[i][0],points_list[i][1],points_list[i][2],c = 'r')
for j in range(0, len(core_data['lines'][0]['line_points'])-1):
    ax.plot3D([core_data['lines'][0]['line_points'][j][0], core_data['lines'][0]['line_points'][j + 1][0]],
              [core_data['lines'][0]['line_points'][j][1], core_data['lines'][0]['line_points'][j + 1][1]],
              [core_data['lines'][0]['line_points'][j][2], core_data['lines'][0]['line_points'][j + 1][2]], 'y',linewidth = 3)
for i in range(0,len(core_data['lanes'])):
    for j in range(0,len(core_data['lanes'][i]['lane_points'])):
        ax.scatter3D(core_data['lanes'][i]['lane_points'][j][0], core_data['lanes'][i]['lane_points'][j][1],
                     core_data['lanes'][i]['lane_points'][j][2],c='w',marker='.')
# for i in range(1,len(core_data['lines'])):
#     for j in range(0,len(core_data['lines'][i]['line_points'])):
#         ax.scatter3D(core_data['lines'][i]['line_points'][j][0], core_data['lines'][i]['line_points'][j][1],
#                      core_data['lines'][i]['line_points'][j][2],c='w')
for i in range(1,len(core_data['lines'])):
    for j in range(0,len(core_data['lines'][i]['line_points'])-1):
        ax.plot3D([core_data['lines'][i]['line_points'][j][0], core_data['lines'][i]['line_points'][j+1][0]],
                  [core_data['lines'][i]['line_points'][j][1], core_data['lines'][i]['line_points'][j + 1][1]],
                    [core_data['lines'][i]['line_points'][j][2], core_data['lines'][i]['line_points'][j + 1][2]],'w',linewidth = 3)
# for m in range(0,len(core_data['lanes'][0]['trajectory'])):
#     for n in range(0,len(core_data['lanes'][0]['trajectory'][m]['traj_points'])):
#         ax.scatter3D(core_data['lanes'][0]['trajectory'][m]['traj_points'][n][0],core_data['lanes'][0]['trajectory'][m]['traj_points'][n][1],
#                      core_data['lanes'][0]['trajectory'][m]['traj_points'][n][2],c = 'k',marker='.')
ax.set_zlabel('Z')
ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.set_xlim(-80, 80)
ax.set_ylim(-60, 60)
ax.set_zlim(-60, 80)
ax.grid(False)
plt.show()



