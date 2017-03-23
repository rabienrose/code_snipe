import random
import trafficsignfunction
import slamfunction
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

width = 1024
height = 768
view_angle = 120
k = slamfunction.calinter(height,width,view_angle)
core_data={}
core_data['width'] = width
core_data['height'] = height
core_data['view_angle'] = view_angle
core_data['frames'] = {}
core_data['mps'] = {}
core_data['trafficsign'] = {}

radius = 20
center1 = [0,0,0]
trafficsign_count = 17
frame_count = 20
trafficpoints = []
trafficsigns = core_data['trafficsign']
pose_list = slamfunction.gen_circle_traj(radius, frame_count)
for i in range(0, trafficsign_count):
    trafficsign = {}
    trafficsign['ROI'] = []
    x = random.uniform(-25, 25)
    z = random.uniform(-25, 25)
    y = random.uniform(-1, 1)
    # x = 10
    # y = 2
    # z = 10
    theta_x = 0
    theta_y = random.uniform(0, 90)
    theta_z = 0
    center = [x,y,z]
    trafficsign['pos'] = center
    trafficsign['type'] = 1
    trafficsign['confidence'] = 1
    pose_t = slamfunction.create_pose(x, y, z, theta_x, theta_y, theta_z)
    twidth = random.randint(3, 5)
    theight = random.randint(3, 5)
    trafficsign['orientation'] = pose_t
    trafficsign['size'] = [twidth,theight]
    trafficpoint = trafficsignfunction.drawtrafficsign(center1, twidth, theight)
    points = slamfunction.transform_point3d(trafficpoint, pose_t)
    trafficpoints.extend(points)
    trafficsign['3dpoints'] = points
    frame_lists = []
    for j in range(0,len(pose_list)):
        ids = []
        uvs,ids = slamfunction.project_3d(k, pose_list[j], points, width, height, ids)
        if len(uvs) == 4:
            frame_lists.append(j)
            x_max = max(uvs[0][0],uvs[1][0],uvs[2][0],uvs[3][0])
            x_min = min(uvs[0][0],uvs[1][0],uvs[2][0],uvs[3][0])
            y_max = max(uvs[0][1],uvs[1][1],uvs[2][1],uvs[3][1])
            y_min = min(uvs[0][1],uvs[1][1],uvs[2][1],uvs[3][1])
            w = x_max - x_min
            h = y_max - y_min
            left_top = [x_min,y_max]
            roi = {}
            roi['frame_idx'] = j
            roi['left_top'] = left_top
            roi['width'] = w
            roi['height'] = h
            roi['uvs'] = uvs
            trafficsign['ROI'].append(roi)
    if frame_lists:
        trafficsign['framerange'] = [min(frame_lists),max(frame_lists)]
        trafficsign['frame_lists'] = frame_lists
    else:
        trafficsign['framerange'] =  []
        trafficsign['frame_lists'] = []
        print('all the keyframe can not see the trafficsign'+str(i))
    trafficsigns[i] = trafficsign
trafficsigns['trafficpoints'] = trafficpoints
trafficsigns['trafficnumbers']= trafficsign_count

# plt,ax = trafficsignfunction.draw3D()
# for j in range(0,frame_count):
#     frame_id = j
#     trafficsignfunction.show_trafficsign(plt,ax,core_data,frame_id,pose_list)

# plt,ax = trafficsignfunction.draw2D()
# for j in range(0,frame_count):
#     frame_id = j
#     trafficsignfunction.trafficsignimage(plt,ax,core_data,frame_id)


