import sys
import os
import logging
import matplotlib.pyplot as plt
import subprocess
import numpy as np
import csv
import shutil
colorLine =['#ff0000', '#990000', '#550000', '#000000']
#colorLine =[(1.0, 0.0, 0.0), (0.75, 0.0, 0.0), (0.5, 0.0, 0.0), (0.25, 0.0, 0.0)]

def plotCase(rootAddrs):
    markCount = 10
    if len(rootAddrs)<=0:
        print("plot_traj_fun/plotCase wrong!!")
        return

    fig = plt.figure(figsize=(6, 6))
    x_gps= np.array([])
    y_gps= np.array([])
    with open(rootAddrs[0] + '/gps.txt', newline='') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            items = row[0].split(' ')
            x_gps = np.append(x_gps, float(items[0]))
            y_gps = np.append(y_gps, float(items[1]))
    avi_x = np.mean(x_gps)
    avi_y = np.mean(y_gps)
    x_gps = x_gps - avi_x
    y_gps = y_gps - avi_y
    line_gps, = plt.plot(x_gps, y_gps, 'b')
    line_gps.set_label('GPS')
    frameCount = len(x_gps)
    step = int(frameCount / markCount)
    for i in range(0, frameCount-1):
            if i%step == 0:
                plt.plot(x_gps[i], y_gps[i],'b.')

    n=0
    copy_flag = False
    for rootAddr in rootAddrs:
        x_slam= np.array([])
        y_slam= np.array([])
        if not os.path.exists(rootAddr + '/slam_pose_m.txt'):
            continue
        with open(rootAddr + '/slam_pose_m.txt', newline='') as csvfile:
            spamreader = csv.reader(csvfile)
            for row in spamreader:
                x_slam = np.append(x_slam, float(row[1]))
                y_slam = np.append(y_slam, float(row[3]))

        x_slam = x_slam - avi_x
        y_slam = y_slam - avi_y
        if n==0:
            ss=x_slam - x_gps
            d_x = np.mean(np.abs(x_slam - x_gps))
            d_y = np.mean(np.abs(y_slam - y_gps))
            error= np.sqrt(d_x*d_x+d_y*d_y)
            plt.annotate('Error: '+ str(error),[0,0])
            if error>10:
                copy_flag = True
        line_slam, = plt.plot(x_slam, y_slam, colorLine[n])
        line_slam.set_label('SLAM')
        slam_frame_count = len(x_slam)
        for i in range(0, frameCount-1):
            if i%step == 0:
                if i<slam_frame_count:
                    plt.plot(x_slam[i], y_slam[i],color=colorLine[n],  marker='.')
        n=n+1

    plt.legend()
    #plt.axes().set_aspect('equal')
    plt.savefig(rootAddrs[0] + '/traj.png', dpi=120)
    if copy_flag:
        (parent_addr, filename) = os.path.split(rootAddrs[0])
        (parent_addr, filename) = os.path.split(parent_addr)
        (parent_addr, case_name) = os.path.split(parent_addr)
        (parent_addr, case_group_name) = os.path.split(parent_addr)

        shutil.copyfile(rootAddrs[0] + '/traj.png', parent_addr + '/traj_'+case_name+'.png')


    #fig.clear()
    #plt.show()

def plot(plotpath, branch_info, savepath):
    #Check whether the input path is correct
    compare_num = len(plotpath)
    dirname = []
    case_data = {}
    for i in range(compare_num):
        if(not os.path.exists(plotpath[i])):
            logging.error('Path not exist!')
            sys.exit(1)
        dirname.append(plotpath[i])
    each_casenum = []
    #Store the time data ,keypoints data and mappoints data of all test cases
    for eachcase in dirname:
        outputpath = {}
        case_group_addrs = []
        case_groups = os.listdir(eachcase)
        for case_group in case_groups:
            if(os.path.isdir(eachcase + '/' + case_group) and case_group != 'kml_snapshot'):
                case_group_addrs.append(eachcase + '/' + case_group)
        for case_group_addr in case_group_addrs:
            cases = os.listdir(case_group_addr)
            for case_name in cases:
                case_addr = case_group_addr +'/'+ case_name
                if os.path.isdir(case_addr):
                    if case_name not in case_data.keys():
                        case_data[case_name] = []
                    case_data[case_name].append(case_addr + '/output/')

    for case_name in case_data:
        #for output_addr in case_data[case_name]:
            #subprocess.run(["/Volumes/chamo/working/test_case_rf/script/transfer_to_gps/convert2gps", output_addr])
        plotCase(case_data[case_name])

plotpath =[
    '/Volumes/chamo/dataset/test/cases_2016-11-03-19-14',
    '/Volumes/chamo/dataset/test/cases_2016-11-03-09-18',
    '/Volumes/chamo/dataset/test/cases_2016-11-01-19-04',
    '/Volumes/chamo/dataset/test/cases_2016-10-31-19-14'
]
branch_info =[]
savepath = '/Volumes/chamo/dataset/test'
plot(plotpath, branch_info, savepath)