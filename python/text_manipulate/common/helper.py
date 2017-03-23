import math
from scipy import spatial
import numpy as np
import os
import shutil

def gps2coor(stand_point, gps):
    diff=[0,0,0]
    diff[0] = gps[0] - stand_point[0]
    diff[1] = gps[1] - stand_point[1]
    diff[2] = gps[2] - stand_point[2]
    latitude = stand_point[1] * 3.14159 / 180
    new_coor =[0,0,0]
    new_coor[0] = diff[0]*(111413 * math.cos(latitude) - 94 * math.cos(3 * latitude))
    new_coor[1] = diff[2]
    new_coor[2] = diff[1]*111320
    return new_coor

def read_traj(file_name, stand_point):
    f = open(file_name)
    lines = f.readlines()
    listData = []
    for kml_line_ind in range(0, len(lines)):
        line = lines[kml_line_ind]
        if line[0] == '<':
            continue
        gps_posi = [float(num) for num in line.split(',')]
        if len(stand_point) == 0:
            stand_point = gps_posi
        coor = gps2coor(stand_point, gps_posi)
        tmp = [0, 0]
        tmp[0] = coor[0]
        tmp[1] = coor[2]
        listData.append(tmp)
    return listData, stand_point

def get_centers(trajs):
    centers = []
    for i in range(0, len(trajs)):
        center = np.mean(trajs[i], axis=0)
        if len(trajs[i])<10:
            print('too few kfs!!!:traj_id:'+str(i))
            print(center)
        centers.append(center)
    return centers

def cal_overlay(traj1, traj2, id1, id2):
    my_tree = spatial.KDTree(traj2)
    start1=-1
    start2=-1
    end1=9999
    end2=9999
    state=-1 #-1 not init, 1 found first close point, 2 found second close point, 3 found first miss point
    for i in range(0, len(traj1), 40):
        inds = my_tree.query_ball_point(traj1[i], 15)
        if len(inds)==0:
            if state == -1:
                continue
            if state == 2:
                state = 3
                break
        else:
            if state == -1:
                start1 = i
                start2 = inds[0]
                state = 1
            elif state == 1:
                if start2 >= inds[0]:
                    state = -1
                    break
                state = 2
            elif state ==2:
                end1 = i
                end2 = inds[0]
    if end1 ==9999:
        return -1
    overlay = []
    section1 = {}
    section1['traj_id'] = id1
    section1['traj_start'] = start1
    section1['traj_end'] = end1
    section2 = {}
    section2['traj_id'] = id2
    section2['traj_start'] = start2
    section2['traj_end'] = end2
    overlay.append(section1)
    overlay.append(section2)
    return overlay

def show_move_files(inds, name_list, root):
    save_addr = root+'re_files/'
    if not os.path.exists(save_addr):
        os.makedirs(save_addr)
    sel_name_list=[]
    for ind in inds:
        base_name = os.path.splitext(name_list[ind])[0]
        print(base_name)
        sel_name_list.append(base_name)
        shutil.copy(root+name_list[ind], save_addr+name_list[ind])
    return sel_name_list

def getTrajIds(core_data, traj_names):
    ids=[]
    all_names = core_data['trajs_name']
    for traj_name in traj_names:
        for i in range(0, len(all_names)):
            base_name = os.path.splitext(all_names[i])[0]
            if base_name == traj_name:
                ids.append(i)
                break
    return ids




