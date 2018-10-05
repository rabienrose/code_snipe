import slamfunction
import numpy as np

frame_count=20
width = 1024
height = 768
view_angle = 127
core_data={}
core_data['width'] = width
core_data['height'] = height
core_data['view_angle'] = view_angle
core_data['frames'] = {}
core_data['mps'] = {}
theta_x = 0
theta_y = 90
theta_z = 0
cen_x =0
cen_y=0
cen_z=0

box_count= 1
length = 5
center = [0,0,0]
N = 2

point_3ds = slamfunction.draw_box(length,center,N)

mps = core_data['mps']
for i in range(0, len(point_3ds)):
    mp={}
    mp['posi'] = point_3ds[i]
    mp['color'] = '#00ff00'
    mps[i] = mp

pose_list = []
pose = slamfunction.create_pose(cen_x, cen_y, cen_z,theta_x, theta_y, theta_z )
pose_list.append(pose)
#pose_list.append(np.matrix(np.identity(4, np.float)))
frames = core_data['frames']
i=0
for pose in pose_list:
    i=i+1
    ids =[]
    frame ={}
    frame['pose'] = pose
    frame['kps'] = []
    frames[i] = frame
plt3d,ax = slamfunction.draw3D()
slamfunction.show_landmark(plt3d,ax,core_data,frame_id=1)