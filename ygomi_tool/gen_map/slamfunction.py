#encoding=utf-8
import math
import numpy as np
import random
from matplotlib.pyplot import savefig
import core_data_helper

#return the list of 3d points
def draw_box(length,center,N):
    posi_list=[]
    posi_list.append([center[0] + length / 2, center[1] + length / 2, center[2] - length / 2])
    posi_list.append([center[0] + length / 2, center[1] - length / 2, center[2] - length / 2])
    posi_list.append([center[0] + length / 2, center[1] + length / 2, center[2] + length / 2])
    posi_list.append([center[0] + length / 2, center[1] - length / 2, center[2] + length / 2])
    posi_list.append([center[0] - length / 2, center[1] - length / 2, center[2] + length / 2])
    posi_list.append([center[0] - length / 2, center[1] + length / 2, center[2] + length / 2])
    posi_list.append([center[0] - length / 2, center[1] + length / 2, center[2] - length / 2])
    posi_list.append([center[0] - length / 2, center[1] - length / 2, center[2] - length / 2])
    posi_tmp = insert_node(posi_list[0],posi_list[1],N)
    posi_tmp.extend(insert_node(posi_list[0], posi_list[2], N))
    posi_tmp.extend(insert_node(posi_list[0], posi_list[6], N))
    posi_tmp.extend(insert_node(posi_list[1], posi_list[3], N))
    posi_tmp.extend(insert_node(posi_list[1], posi_list[7], N))
    posi_tmp.extend(insert_node(posi_list[2], posi_list[3], N))
    posi_tmp.extend(insert_node(posi_list[2], posi_list[5], N))
    posi_tmp.extend(insert_node(posi_list[5], posi_list[4], N))
    posi_tmp.extend(insert_node(posi_list[5], posi_list[6], N))
    posi_tmp.extend(insert_node(posi_list[4], posi_list[7], N))
    posi_tmp.extend(insert_node(posi_list[6], posi_list[7], N))
    posi_tmp.extend(insert_node(posi_list[4], posi_list[3], N))
    posi_list.extend(posi_tmp)
    return posi_list

def insert_node(p1,p2,N):
    a = np.array(p1)
    b = np.array(p2)
    dir = b - a
    dir_norm = np.linalg.norm(dir)
    dir = dir / dir_norm
    step_length = dir_norm / N
    point_lists = []
    for i in range(1,N):
        point_tmp = p1 + i * step_length*dir
        point_lists.append(point_tmp.tolist())
    return point_lists

def show_camera(ax,pose, viewangle,ctype):
    angle = viewangle / 360.0 * 2 * 3.1415926
    angle = angle / 2
    rot = pose[0:3, 0:3]
    t = pose[0:3, 3]
    direction = rot.transpose()
    posi = -direction * t
    dir_x = direction[0:3, 0]
    dir_norm = np.linalg.norm(dir_x)
    dir_x = dir_x / dir_norm
    dir_y = direction[0:3, 1]
    dir_norm = np.linalg.norm(dir_y)
    dir_y = dir_y / dir_norm
    dir_z = direction[0:3, 2]
    dir_norm = np.linalg.norm(dir_z)
    dir_z = dir_z / dir_norm
    scale = 1

    width = scale * math.tan(angle)
    scale1 = width
    pt1 = posi + dir_z * scale*3
    pt2 = posi + dir_x * width + dir_z * scale
    pt3 = posi - dir_x * width + dir_z * scale
    pt4 = pt2 - dir_y * scale1
    pt7 = pt2 + dir_y * scale1
    pt5 = pt3 - dir_y * scale1
    pt6 = pt3 + dir_y * scale1
    ax.plot3D([posi[0, 0], pt1[0, 0]], [posi[1, 0], pt1[1, 0]], [posi[2, 0], pt1[2, 0]], ctype)
    ax.plot3D([posi[0, 0], pt4[0, 0]], [posi[1, 0], pt4[1, 0]], [posi[2, 0], pt4[2, 0]], ctype)
    ax.plot3D([posi[0, 0], pt5[0, 0]], [posi[1, 0], pt5[1, 0]], [posi[2, 0], pt5[2, 0]], ctype)
    ax.plot3D([posi[0, 0], pt6[0, 0]], [posi[1, 0], pt6[1, 0]], [posi[2, 0], pt6[2, 0]], ctype)
    ax.plot3D([posi[0, 0], pt7[0, 0]], [posi[1, 0], pt7[1, 0]], [posi[2, 0], pt7[2, 0]], ctype)
    ax.plot3D([pt4[0, 0], pt5[0, 0]], [pt4[1, 0], pt5[1, 0]], [pt4[2, 0], pt5[2, 0]], ctype)
    ax.plot3D([pt4[0, 0], pt7[0, 0]], [pt4[1, 0], pt7[1, 0]], [pt4[2, 0], pt7[2, 0]], ctype)
    ax.plot3D([pt6[0, 0], pt7[0, 0]], [pt6[1, 0], pt7[1, 0]], [pt6[2, 0], pt7[2, 0]], ctype)
    ax.plot3D([pt6[0, 0], pt5[0, 0]], [pt6[1, 0], pt5[1, 0]], [pt6[2, 0], pt5[2, 0]], ctype)
    #ax.plot3D(posi[0], posi[1], posi[2], color='y', marker ='.')

def show_frameid_camera(ax, core_data, frame_id,camera_FLAG):
    poselist =core_data_helper.get_pose_list(core_data, range(0, len(core_data['frames'])))
    if camera_FLAG:
        for i in range(0,len(core_data['lanes'])):
            if 'trajectory' not in core_data['lanes'][i]:
                continue
            for m in range(0,len(core_data['lanes'][i]['trajectory'])):
                centers = []
                for j in range(0,len(core_data['lanes'][i]['trajectory'][m]['frame_id'])):
                    rot = poselist[core_data['lanes'][i]['trajectory'][m]['frame_id'][j]][0:3, 0:3]
                    t = poselist[core_data['lanes'][i]['trajectory'][m]['frame_id'][j]][0:3, 3]
                    direction = rot.transpose()
                    posi = -direction * t  # 求相机的中心点，连接相机能看到的traffic sign
                    centers.append(posi)
                for n in range(0,len(centers)-1):
                    ax.plot3D([centers[n][0,0],centers[n+1][0,0]],[centers[n][1,0],centers[n+1][1,0]],[centers[n][2,0],centers[n+1][2,0]],'g')
    for i in range(0, len(poselist)):
        if i == frame_id:
            show_camera(ax, poselist[i], core_data['view_angle'], 'r')
        else:
            show_camera(ax, poselist[i], core_data['view_angle'], 'b')

def show_image(ax, core_data, frame_id):
    for i in range(0, len(core_data['frames'])):
        frame = core_data['frames'][i]
        if 'mp_list' not in frame:
            continue
        mp_list = frame['mp_list']
        for j in range(0, len(mp_list)):
            track = core_data['mps'][mp_list[j]]['track']
            for k in range(0, len(track)):
                if track[k]['frame_id']== frame_id:
                    uv = track[k]['uv']
                    color = core_data['mps'][mp_list[j]]['color']
                    ax.plot(uv[0], uv[1], color=color, marker='.')

def show_landmark(ax,core_data):
    for i in range(0, len(core_data['mps'])):
        color = core_data['mps'][i]['color']
        mpoints = core_data['mps'][i]['posi']
        ax.scatter(mpoints[0], mpoints[1], mpoints[2], color=color)

#to-do: modify to work with new core_data
def showtrack(ax,core_data,frame_id):
    mp_ids = []
    kp_list = core_data['frames'][frame_id]['kps']
    for kp_id in range(0,len(kp_list)):
        mp_id = kp_list[kp_id]['mp_id']
        tracks = core_data['mps'][mp_id]['track']
        track_uv = []
        find_kp = False
        for item in tracks:
            if item[0] == frame_id and item[1] == kp_id:
                track_uv.append(core_data['frames'][item[0]]['kps'][item[1]]['uv'])
                find_kp= True
        if find_kp == False:
            print('cannot find kp in track!!')
            continue
        find_kp = False
        for item in tracks:
            if item[0] == frame_id-1:
                track_uv.append(core_data['frames'][item[0]]['kps'][item[1]]['uv'])
                find_kp = True
        if find_kp == False:
            continue
        ax.plot(track_uv[0][0],track_uv[0][1],'r.')
        ax.plot([track_uv[0][0], track_uv[1][0]],[track_uv[0][1], track_uv[1][1]],'b')
    ax.set_aspect('equal')
    #plt.show()
    address = '/Users/test/Documents/generate_map/script/gen_map/track_image/track_' + str(frame_id) + '.png'
    savefig(address)

def generate_landmark(core_data, landmark_count,road_bone,road_num,lane_width):
    N = 3
    box_center = [0, 0, 0]
    length = 5
    points_box = draw_box(length, box_center, N)
    if 'mps' not in core_data.keys():
        core_data['mps'] =[]
    mps = core_data['mps']
    for i in range(0, landmark_count):
        pick_num = random.randint(0, road_num)    #500
        road_coor = road_bone[pick_num]
        x = random.uniform(road_coor[0] - lane_width*2, road_coor[0] + lane_width*2)
        z = random.uniform(road_coor[2] - lane_width*2, road_coor[2] + lane_width*2)
        y = random.uniform(-10, 10)
        theta_x = 0
        theta_y = random.uniform(0, 360)
        theta_z = 0
        pose_t = core_data_helper.create_pose(x, y, z, theta_x, theta_y, theta_z)
        points = core_data_helper.transform_point3d(points_box, pose_t)
        color = core_data_helper.get_rand_color()
        for j in range(0, len(points)):
            mp = {}
            mp['id'] = len(mps)+1
            mp['posi'] = points[j]
            mp['color'] = color
            mps.append(mp)

def add_descriptor(core_data,filename):
    des = []
    f = open(filename + 'output/' + 'descriptor.txt')
    lines = f.readlines()
    for line in lines:
        dess = [float(i) for i in line.split()]
        des.append(dess)
    for i in range(0,len(core_data['mps'])):
        core_data['mps'][i]['des'] = des[i]

def project_landmark(core_data, frame_id_list):
    if 'mps' not in core_data.keys():
        return
    if len(frame_id_list) == 0:
        return
    width = core_data['width']
    height = core_data['height']
    view_angle = core_data['view_angle']
    k = core_data_helper.cal_k(height, width, view_angle)
    mps = core_data['mps']
    point_3ds = []
    for i in range(0, len(mps)):
        point_3ds.append(mps[i]['posi'])
    frames=[]
    for i in range(0, len(frame_id_list)):
        frames.append(core_data['frames'][frame_id_list[i]])
    for i in range(0, len(frames)):
        pose = np.matrix(frames[i]['pose'])
        uvs, ids = core_data_helper.project_3d(k, pose , point_3ds, width, height)  # mps
        for j in range(0, len(uvs)):
            if 'mp_list' not in frames[i]:
                frames[i]['mp_list'] = []
            frames[i]['mp_list'].append(ids[j])
            track_item = {}
            track_item['frame_id'] = i
            track_item['uv'] = uvs[j]
            if 'track' not in mps[ids[j]]:
                mps[ids[j]]['track'] = []
            mps[ids[j]]['track'].append(track_item)
