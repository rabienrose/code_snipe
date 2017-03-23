
import matplotlib.pyplot as plt
import numpy as np
import math
from scipy import spatial
from os import listdir
from os.path import isfile, join
import sys
import os
import time
import copy
import pickle
import common.helper
import subprocess
import shutil
sys.setrecursionlimit(100000)

save_re = 9
# 1:call overlay
# 2:show result
# 3:savefile
# 4:cal overlap dis
# 5:run slam
# 6:cal spec traj overlap
raw_gps_addr = '/Volumes/chamo/dataset/gm_new/kml/'
if save_re ==1:
    core_data={}
    onlyfiles = [f for f in listdir(raw_gps_addr) if isfile(join(raw_gps_addr, f))]
    stand_point=[]
    trajs=[]
    traj_name=[]
    for i in range(0, len(onlyfiles)):
        if onlyfiles[i] == '.DS_Store':
            continue
        traj, stand_point= common.helper.read_traj(raw_gps_addr+onlyfiles[i], stand_point)
        trajs.append(traj)
        traj_name.append(onlyfiles[i])
    centers = common.helper.get_centers(trajs)
    center_tree = spatial.KDTree(centers)
    for i in range(0, len(trajs)):
        inds = center_tree.query_ball_point(centers[i], 1500)
        for j in inds:
            if i==j:
                continue
            if j in core_data:
                if i in core_data[j]:
                    if i not in core_data:
                        core_data[i] = {}
                    core_data[i][j]= core_data[j][i]
                    continue
            re = common.helper.cal_overlay(trajs[i], trajs[j], i, j)
            if re ==-1:
                continue
            if i not in core_data:
                core_data[i] = {}
            core_data[i][j] = re
    core_data['trajs_name'] = traj_name
    f = open("temp_core", 'wb')
    pickle.dump(core_data, f)
    f.close()
elif save_re ==2:
    f = open("temp_core", 'rb')
    core_data = pickle.load(f)
    f.close()
    overlay_counts=[]
    for i in core_data:
        overlay_counts.append([i, len(core_data[i])])
    print(overlay_counts)
elif save_re ==3:
    f = open("temp_core", 'rb')
    core_data = pickle.load(f)
    f.close()
    save_traj_id = 436
    ids=[]
    for traj_id in core_data[save_traj_id]:
        ids.append(traj_id)
    namelist =common.helper.show_move_files(ids, core_data['trajs_name'], raw_gps_addr)
    f = open("name_list1", 'wb')
    pickle.dump(namelist, f)
    f.close()
elif save_re ==4:
    f = open("temp_core", 'rb')
    core_data = pickle.load(f)
    f.close()
    tar_traj_id = 436
    max_kf = 1800
    overlap_dis={}
    for i in range(0, max_kf):
        for traj_id in core_data[tar_traj_id]:
            overlap = core_data[tar_traj_id][traj_id]
            for j in range(0, len(overlap)):
                if overlap[j]['traj_id'] == tar_traj_id:
                    start_kf = overlap[j]['traj_start']
                    end_kf = overlap[j]['traj_end']
                    if i <= end_kf and i >= start_kf:
                        if i not in overlap_dis:
                            overlap_dis[i]=[]
                        overlap_dis[i].append(traj_id)
                    break
    overlap_count_dis=[]
    for i in overlap_dis:
        print(str(i)+':'+str(len(overlap_dis[i])))
    name_list = common.helper.show_move_files(overlap_dis[1100], core_data['trajs_name'], raw_gps_addr)
    f = open("name_list", 'wb')
    pickle.dump(name_list, f)
    f.close()
elif save_re ==5:
    f = open("name_list", 'rb')
    name_list = pickle.load(f)
    f.close()
    for i in range(0, len(name_list)):
        exe_addr ='/Volumes/chamo/working/server_repo/new_vehicle/examples/Monocular/mono_kitti'
        gps_addr = '/Volumes/chamo/dataset/gm_new/ref/'+name_list[i]+'.txt'
        image_addr = '/Volumes/chamo/dataset/gm_new/images/'+name_list[i]
        voc_addr = '/Volumes/chamo/working/server_repo/new_vehicle/bin/resources/SGDvoc.txt'
        cfg_addr = '/Volumes/chamo/working/server_repo/new_vehicle/bin/resources/CONTI/CONTI65.yaml'
        bashCommand = exe_addr + ' ' + voc_addr + ' ' + cfg_addr + ' ' + image_addr + ' ' + gps_addr
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        print(bashCommand)
elif save_re ==6:
    traj_names = ['2016-11-16_T_20-08-00.781_GMT_cut1', '2016-12-02_T_21-16-36.259_GMT_cut0']
    f = open("temp_core", 'rb')
    core_data = pickle.load(f)
    f.close()
    ids = common.helper.getTrajIds(core_data, traj_names)
    overlap = core_data[ids[0]][ids[1]]
    print(core_data['trajs_name'][overlap[0]['traj_id']])
    print(core_data['trajs_name'][overlap[1]['traj_id']])
    print(overlap)
elif save_re == 7:
    f = open("name_list", 'rb')
    name_list = pickle.load(f)
    f.close()
    root = '/Volumes/chamo/dataset/gm_new/'
    for i in range(0, len(name_list)):
        base_name = name_list[i]
        source=root + 'rtv/' + base_name+'.rtv'
        dist = root + 'rtv1/' + base_name+'.rtv'
        shutil.copy(source, dist)
        source = root + 'kml/' + base_name + '.kml'
        dist = root + 'kml1/' + base_name + '.kml'
        shutil.copy(source, dist)
elif save_re == 8:
    f = open("temp_core", 'rb')
    core_data = pickle.load(f)
    f.close()
    print(core_data['trajs_name'][436])
elif save_re == 9:
    root = '/Volumes/chamo/dataset/gm_new/'
    file_list_folder =root+'shared_traj2/kml1'
    onlyfiles = [f for f in listdir(file_list_folder) if isfile(join(file_list_folder, f))]
    for i in range(0, len(onlyfiles)):
        if onlyfiles[i] == '.DS_Store':
            continue
        base_name = os.path.splitext(onlyfiles[i])[0]
        source = root + 'images/' + base_name
        dist = root + 'shared_traj2/image/' + base_name
        shutil.copytree(source, dist)






