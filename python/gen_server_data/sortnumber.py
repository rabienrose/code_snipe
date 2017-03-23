#encoding=utf-8
import copy
import core_data_helper
import core_data_obj
import os
import random
import numpy as np
def sort_number(core_data, filename):
    if not os.path.exists(filename):
        os.makedirs(filename)
    out = 0
    for i in range(0,len(core_data['lanes'])):                                                   #trajectory
        if 'trajectory' not in core_data['lanes'][i]:
            continue
        traj_total_number = len(core_data['lanes'][i]['trajectory'])                         #统计每个车道轨迹数量
        for j in range(0,traj_total_number):
            xx = 0
            yy = 0
            zz = random.uniform(-1, 1)
            theta_x = 0
            theta_y = 0
            theta_z = 0
            T = CalTransform(xx,yy,zz,theta_x,theta_y,theta_z)                            #gps轨迹与SLAM轨迹变换矩阵
            core_data1 = copy.deepcopy(core_data)
            traj_list = core_data['lanes'][i]['trajectory'][j]['frame_id']
            core_data1['traj_list'] = traj_list
            core_data1['lanes'][i]['trajectory'] = []
            for iii in range(0,len(core_data['lanes'])):
                if iii != i and 'trajectory' in core_data1['lanes'][iii]:
                    del core_data1['lanes'][iii]['trajectory']
            tr = copy.deepcopy(core_data['lanes'][i]['trajectory'][j])    #拷贝当前要处理的轨迹
            core_data1['lanes'][i]['trajectory'].append(tr)
            basenumber = core_data['lanes'][i]['trajectory'][j]['frame_id'][0]  #记录当前轨迹起始编号
            for m in range(0,len(core_data1['lanes'][i]['trajectory'][0]['frame_id'])):
                core_data1['lanes'][i]['trajectory'][0]['frame_id'][m] = core_data1['lanes'][i]['trajectory'][0]['frame_id'][m] - basenumber

            core_data1['trafficsign'] = []                                                      #traffic sign
            for m in range(0,len(core_data['trafficsign'])):
                current_traffic = copy.deepcopy(core_data['trafficsign'][m])
                current_traffic['ROI'] = []
                center = current_traffic['center']
                current_traffic['center'] = Transcoor(center,T)         #变换traffic sign的center
                vertice = current_traffic['vertice']
                current_traffic['vertice'] = []
                for kkk in range(0,4):
                    current_traffic['vertice'].append(Transcoor(vertice[kkk],T))
                if 'ROI'in core_data['trafficsign'][m]:
                    for n in range(0,len(core_data['trafficsign'][m]['ROI'])):                   #调整traffic sign的共视关系
                        if core_data['trafficsign'][m]['ROI'][n]['frame_id'] in traj_list:
                            tt = copy.deepcopy(core_data['trafficsign'][m]['ROI'][n])
                            tt['frame_id']=tt['frame_id'] - basenumber
                            current_traffic['ROI'].append(tt)
                if current_traffic['ROI']:
                    core_data1['trafficsign'].append(current_traffic)

            core_data1['mps'] = []
            ii = 0
            for m in range(0,len(core_data['mps'])):                                          #mps
                current_mps = copy.deepcopy(core_data['mps'][m])
                posi = current_mps['posi']
                current_mps['posi'] = Transcoor(posi,T)                       #trans mp's center
                current_mps['track'] = []
                if 'track' in core_data['mps'][m]:                           #调整mps的共视关系
                    for n in range(0,len(core_data['mps'][m]['track'])):
                        if core_data['mps'][m]['track'][n]['frame_id'] in traj_list:
                            mm = copy.deepcopy(core_data['mps'][m]['track'][n])
                            mm['frame_id'] = mm['frame_id'] - basenumber
                            current_mps['track'].append(mm)
                if current_mps['track']:
                    current_mps['id'] = ii
                    ii = ii + 1
                    core_data1['mps'].append(current_mps)

            core_data1['frames'] = []
            for m in range(0,len(core_data['frames'])):
                current_frames = copy.deepcopy(core_data['frames'][m])
                if core_data['frames'][m]['id'] in traj_list:
                    current_frames['id'] = current_frames['id'] - basenumber
                    gpspose = np.matrix(current_frames['pose'])
                    # y1 = core_data_helper.checkr(gpspose)                 #同步server端检测pose是否符合要求
                    gpspose = gpspose.I
                    gpspose1 = T*gpspose
                    gpspose1 = normr(gpspose1)
                    SLAMpose = gpspose1.I                                   #求SLAM轨迹
                    # y2 = core_data_helper.checkr(SLAMpose)
                    current_frames['slampose'] = SLAMpose.tolist()                       #change to slampose
                    core_data1['frames'].append(current_frames)

            core_data1['lines'] = []
            for m in range(0,len(core_data['lines'])):
                current_lines = copy.deepcopy(core_data['lines'][m])
                for kkk in range(0,len(current_lines['line_points'])):              #trans
                    coor = current_lines['line_points'][kkk]
                    current_lines['line_points'][kkk] = Transcoor(coor,T)
                current_lines['images'] = []
                if 'images' in core_data['lines'][m]:
                    for n in range(0,len(core_data['lines'][m]['images'])):
                        if core_data['lines'][m]['images'][n]['frame_id'] in traj_list:
                            ll = copy.deepcopy(core_data['lines'][m]['images'][n])
                            ll['frame_id'] = ll['frame_id'] - basenumber
                            current_lines['images'].append(ll)
                core_data1['lines'].append(current_lines)
            # core_data_obj.display_scene(core_data1, 1, 120)                  #用于显示变换之后的SLAM轨迹
            for m in range(0,len(core_data['lines'])):
                core_data1['lines'][m]['line_points'] = []
                for n in range(0,len(core_data['lines'][m]['line_points'])):
                    line_point={}
                    kf_range = []
                    for nn in range(traj_list[0],traj_list[-1]):
                        if n in core_data['frames'][nn]['line_list'][m]['line_pt_id']:
                            kf_range.append(nn-basenumber)
                    if len(kf_range) == 0:
                        kf_range.append(-1)
                    line_point['kf_range'] = kf_range
                    line_point['id'] = n
                    line_point['coor'] = core_data['lines'][m]['line_points'][n]
                    core_data1['lines'][m]['line_points'].append(line_point)

            core_data_helper.save_result(core_data1, filename + 'output' + str(out) + '.txt')
            out = out + 1


# def CalTransform(x,y,z,theta_x,theta_y,theta_z):                        # s 乘在R t外面
#     pose = core_data_helper.create_pose(x,y,z,theta_x,theta_y,theta_z)
#     scale = np.matrix(np.identity(4, np.float))
#     s_x = 1.1
#     s_y = 1.1
#     s_z = 1.1
#     scale[0,0] = s_x
#     scale[1,1] = s_y
#     scale[2,2] = s_z
#     Trans = scale * pose
#     return Trans


def Transcoor(coor_old,T):
    mp_tmp = np.matrix(np.ones((4, 1), np.float))
    mp_tmp[0, 0] = coor_old[0]
    mp_tmp[1, 0] = coor_old[1]
    mp_tmp[2, 0] = coor_old[2]
    mp_new = T*mp_tmp
    coor_new = []
    coor_new.append(mp_new[0,0])
    coor_new.append(mp_new[1,0])
    coor_new.append(mp_new[2,0])
    return coor_new
def CalTransform(x,y,z,theta_x,theta_y,theta_z):                         # s只乘R
    pose = core_data_helper.create_pose(x,y,z,theta_x,theta_y,theta_z)
    rot = pose[0:3, 0:3]
    t = pose[0:3, 3]
    scale = np.matrix(np.identity(3, np.float))
    Trans = np.matrix(np.identity(4, np.float))
    s_x = 1
    s_y = 1
    s_z = 1
    scale[0,0] = s_x
    scale[1,1] = s_y
    scale[2,2] = s_z
    rot1 = scale * rot
    Trans[0:3, 0:3] = rot1
    Trans[0:3, 3] = t
    return Trans

def normr(pose):                        # 归一化朝向
    dir_x = pose[0:3, 0]
    dir_norm = np.linalg.norm(dir_x)
    dir_x = dir_x / dir_norm
    dir_y = pose[0:3, 1]
    dir_norm = np.linalg.norm(dir_y)
    dir_y = dir_y / dir_norm
    dir_z = pose[0:3, 2]
    dir_norm = np.linalg.norm(dir_z)
    dir_z = dir_z / dir_norm
    pose[0:3, 0] = dir_x
    pose[0:3, 1] = dir_y
    pose[0:3, 2] = dir_z
    return pose

def calcenter(pose):
    rot = pose[0:3, 0:3]
    t = pose[0:3, 3]
    direction = rot.transpose()
    posi = -direction * t
    return posi











