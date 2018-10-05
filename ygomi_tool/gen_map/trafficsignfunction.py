#encoding=utf-8
import core_data_helper
import random
import numpy as np

def drawtrafficsign(center,width,height):
    posi_list = []
    posi_list.append([center[0] + width / 2,center[1] + height / 2,center[2]])
    posi_list.append([center[0] + width / 2,center[1] - height / 2,center[2]])
    posi_list.append([center[0] - width / 2,center[1] - height / 2,center[2]])
    posi_list.append([center[0] - width / 2,center[1] + height / 2,center[2]])
    return posi_list

def show_trafficsign(ax,core_data,frame_id):
    if not 'trafficsign' in core_data:
        return
    pose = np.matrix(core_data['frames'][frame_id]['pose'])
    rot = pose[0:3, 0:3]
    t = pose[0:3, 3]
    direction = rot.transpose()
    posi = -direction * t                     #求相机的中心点，连接相机能看到的traffic sign
    for j in range(0,len(core_data['trafficsign'])):
        points = core_data['trafficsign'][j]['vertice']
        ax.plot3D([points[0][0], points[1][0]], [points[0][1], points[1][1]], [points[0][2], points[1][2]], 'b')
        ax.plot3D([points[1][0], points[2][0]], [points[1][1], points[2][1]], [points[1][2], points[2][2]], 'b')
        ax.plot3D([points[2][0], points[3][0]], [points[2][1], points[3][1]], [points[2][2], points[3][2]], 'b')
        ax.plot3D([points[3][0], points[0][0]], [points[3][1], points[0][1]], [points[3][2], points[0][2]], 'b')
        for k in range(0, len(core_data['trafficsign'][j]['ROI'])):
            if frame_id == core_data['trafficsign'][j]['ROI'][k]['frame_id']:
                center = core_data['trafficsign'][j]['center']
                ax.plot3D([center[0],posi[0]],[center[1],posi[1]],[center[2],posi[2]],'k')

def trafficsignimage(ax,core_data,frame_id):
    if not 'trafficsign' in core_data:
        return
    for i in range(0,len(core_data['trafficsign'])):
        trafficsign = core_data['trafficsign'][i]
        for j in range(0,len(trafficsign['ROI'])):
            roi = trafficsign['ROI'][j]
            if frame_id == roi['frame_id']:
                uvs = roi['uvs']
                left_top = roi['bound_ax']
                wid = roi['bound_w']
                hei = roi['bound_h']
                ax.plot([uvs[0][0], uvs[1][0]], [uvs[0][1], uvs[1][1]],'r')
                ax.plot([uvs[1][0], uvs[2][0]], [uvs[1][1], uvs[2][1]],'r')
                ax.plot([uvs[2][0], uvs[3][0]], [uvs[2][1], uvs[3][1]],'r')
                ax.plot([uvs[3][0], uvs[0][0]], [uvs[3][1], uvs[0][1]],'r')
                right_top = [left_top[0]+wid,left_top[1]]
                left_down = [left_top[0],left_top[1]-hei]
                right_down = [left_top[0]+wid,left_top[1]-hei]
                ax.plot([left_top[0], right_top[0]], [left_top[1], right_top[1]], 'y')
                ax.plot([right_top[0],right_down[0]],[right_top[1],right_down[1]],'y')
                ax.plot([right_down[0],left_down[0]],[right_down[1],left_down[1]],'y')
                ax.plot([left_down[0],left_top[0]], [left_down[1],left_top[1]],'y')

def generate_ts(core_data, trafficsign_count,road_bone,road_num,lane_width):
    if not 'trafficsign' in core_data:
        core_data['trafficsign'] = []
    traffic_center = [0, 0, 0]
    trafficsigns = core_data['trafficsign']
    for i in range(0, trafficsign_count):
        trafficsign = {}
        trafficsign['ROI'] = []
        pick_num = random.randint(0,road_num)
        road_coor = road_bone[pick_num]
        x = random.uniform(road_coor[0]-lane_width*2, road_coor[0]+lane_width*2)    #depend on the width of the road
        z = random.uniform(road_coor[2]-lane_width*2, road_coor[2]+lane_width*2)
        y = random.uniform(-1, 1)
        theta_x = 0
        theta_y = random.uniform(0, 90)
        theta_z = 0
        center = [x,y,z]
        trafficsign['center'] = center
        pose_t = core_data_helper.create_pose(x, y, z, theta_x, theta_y, theta_z)
        twidth = random.randint(3, 5)
        theight = random.randint(3, 5)
        trafficsign['orientation'] = pose_t.tolist()
        trafficsign['size'] = [twidth,theight]
        trafficpoint = drawtrafficsign(traffic_center, twidth, theight)
        points = core_data_helper.transform_point3d(trafficpoint, pose_t)
        trafficsign['vertice'] = points
        trafficsigns.append(trafficsign)

def project_ts(core_data, frame_id_list):
    if not 'trafficsign' in core_data:
        return
    width = core_data['width']
    height = core_data['height']
    view_angle = core_data['view_angle']
    k = core_data_helper.cal_k(height, width, view_angle)
    trafficsign_count = len(core_data['trafficsign'])
    for i in range(0, trafficsign_count):
        trafficsign = core_data['trafficsign'][i]
        points = trafficsign['vertice']
        frames = []
        for n in range(0, len(frame_id_list)):
            frames.append(core_data['frames'][frame_id_list[n]])
        for j in range(0, len(frames)):
            uvs, ids = core_data_helper.project_3d(k, np.matrix(frames[j]['pose']), points, width, height)
            if len(uvs) == 4:
                if 'ROI' not in trafficsign:
                    trafficsign['ROI']=[]
                x_max = max(uvs[0][0], uvs[1][0], uvs[2][0], uvs[3][0])
                x_min = min(uvs[0][0], uvs[1][0], uvs[2][0], uvs[3][0])
                y_max = max(uvs[0][1], uvs[1][1], uvs[2][1], uvs[3][1])
                y_min = min(uvs[0][1], uvs[1][1], uvs[2][1], uvs[3][1])
                w = x_max - x_min
                h = y_max - y_min
                left_top = [x_min, y_max]
                roi = {}
                roi['frame_id'] = j
                roi['bound_ax'] = left_top
                roi['bound_w'] = w
                roi['bound_h'] = h
                roi['uvs'] = uvs
                trafficsign['ROI'].append(roi)
                if 'ts_list' not in frames[j]:
                    frames[j]['ts_list']=[]
                frames[j]['ts_list'].append(i)
