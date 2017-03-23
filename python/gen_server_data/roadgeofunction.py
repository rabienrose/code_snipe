#encoding=utf-8
import numpy as np
import random
from scipy import interpolate
import core_data_helper

def interpolate3d(points_list,N):
    points_new = []
    x = []
    y = []
    z = []
    for i in range(0,len(points_list)):
        x.append(points_list[i][0])
        y.append(points_list[i][1])
        z.append(points_list[i][2])
    tck, t = interpolate.splprep([x, y, z], s=0)
    xi, yi, zi = interpolate.splev(np.linspace(t[0], t[-1], N), tck)
    for i in range(0,N):
        points_new.append([xi[i],yi[i],zi[i]])
    return points_new

def generateLine(lines,points_list,N,s):
    dir_up = np.array([0, 1, 0])
    for i in range(0,N+1):       #正向扩展
        line = {}
        if i == 0:
            line['line_id'] = i
            line['l_lane_id'] = i
            line['r_lane_id'] = N
            line['line_points'] = points_list
        else:
            line['line_id'] = i
            line['l_lane_id'] = i
            line['r_lane_id'] = i-1
            if i == N:
                line['l_lane_id'] = -1
                line['r_lane_id'] = i - 1
            pointsnew = []
            for j in range(0, len(points_list) - 1):
                dire = [points_list[j + 1][0] - points_list[j][0], points_list[j + 1][1] - points_list[j][1],
                        points_list[j + 1][2] - points_list[j][2]]
                dir_z = np.array(dire)
                dir_x = np.cross(dir_z, dir_up)
                dir_norm = np.linalg.norm(dir_x)
                dir_x = dir_x / dir_norm
                scaling = dir_x * s
                new_p=points_list[j] + scaling*i
                pointsnew.append(new_p.tolist())
            line['line_points'] = pointsnew
        # lines[i] = line
        lines.append(line)
    for i in range(1,N+1):       #逆向扩展
        line = {}
        line['line_id'] = N + i
        line['l_lane_id'] = N + i - 1
        line['r_lane_id'] = N + i
        if i == N:
            line['l_lane_id'] = N + i - 1
            line['r_lane_id'] = -1
        pointsnew = []
        for j in range(0, len(points_list) - 1):
            dire = [points_list[j + 1][0] - points_list[j][0], points_list[j + 1][1] - points_list[j][1],
                    points_list[j + 1][2] - points_list[j][2]]
            dir_z = np.array(dire)
            dir_x = np.cross(dir_z, dir_up)
            dir_norm = np.linalg.norm(dir_x)
            dir_x = dir_x / dir_norm
            scaling = dir_x * s
            new_p = points_list[j] - scaling*i
            pointsnew.append(new_p.tolist())
        line['line_points'] = pointsnew
        # lines[N+i] = line
        lines.append(line)
    return lines

def generateLane(lanes,points_list,N,s):
    dir_up = np.array([0, 1, 0])
    for i in range(0,N):                     #正向扩展
        lane = {}
        lane['lane_id'] = i
        lane['l_line_id'] = i + 1
        lane['r_line_id'] = i
        pointsnew = []
        for j in range(0, len(points_list) - 1):
            dire = [points_list[j + 1][0] - points_list[j][0], points_list[j + 1][1] - points_list[j][1],
                    points_list[j + 1][2] - points_list[j][2]]
            dir_z = np.array(dire)
            dir_x = np.cross(dir_z, dir_up)
            dir_norm = np.linalg.norm(dir_x)
            dir_x = dir_x / dir_norm
            scaling = dir_x * s
            new_p = points_list[j] + scaling * (i * 2 + 1)
            pointsnew.append(new_p.tolist())
        lane['lane_points'] = pointsnew
        # lanes[i] = lane
        lanes.append(lane)
    for i in range(0,N):                     #逆向扩展
        lane = {}
        lane['lane_id'] = N+i
        if i == N:
            lane['l_line_id'] = 0
            lane['r_line_id'] = N+i+1
        else:
            lane['l_line_id'] = N+i
            lane['r_line_id'] = N+i+1
        pointsnew = []
        for j in range(0, len(points_list) - 1):
            dire = [points_list[j + 1][0] - points_list[j][0], points_list[j + 1][1] - points_list[j][1],
                    points_list[j + 1][2] - points_list[j][2]]
            dir_z = np.array(dire)
            dir_x = np.cross(dir_z, dir_up)
            dir_norm = np.linalg.norm(dir_x)
            dir_x = dir_x / dir_norm
            scaling = dir_x * s
            new_p= points_list[j] - scaling*(i*2+1)
            pointsnew.append(new_p.tolist())
        lane['lane_points'] = pointsnew
        # lanes[N+i] = lane
        lanes.append(lane)
    return lanes

def generate_road(core_data, road_bone,half_N,s):
    if 'lines' not in core_data.keys():
        core_data['lines'] =[]
    lines = core_data['lines']
    generateLine(lines, road_bone, half_N, s)
    if 'lanes' not in core_data.keys():
        core_data['lanes'] = []
    lines = core_data['lanes']
    generateLane(lines, road_bone, half_N, s/2)

def generate_veh_traj(core_data,lane_number,traj_number, traj_noise, kf_count,sample_N):
    dir_up = np.array([0, 1, 0])
    traj_smooth = 5    #road_bone每隔几个点取一个点生成轨迹bone
    height = -5              #行车轨迹离开路面的高度
    x = 0           #记录keyframe的编号
    pose_lists = []
    for l in range(0,len(lane_number)):
        lanepoints = core_data['lanes'][lane_number[l]]['lane_points']
        number = int(len(lanepoints) / traj_smooth)
        basepoints = []
        for i in range(0,number):
            basepoints.append(lanepoints[traj_smooth*i])
        basepoints.append(lanepoints[len(lanepoints)-1])
        if 'trajectory' not in core_data['lanes'][lane_number[l]]:
            core_data['lanes'][lane_number[l]]['trajectory'] = []
        trajectorys = core_data['lanes'][lane_number[l]]['trajectory']
        for i in range(0,traj_number[l]):
            trajectory  = {}
            trajectory['frame_id'] = []
            points_tmp = []
            for j in range(0, len(basepoints) - 1):
                dire = [basepoints[j + 1][0] - basepoints[j][0], basepoints[j + 1][1] - basepoints[j][1],
                        basepoints[j + 1][2] - basepoints[j][2]]
                dir_z = np.array(dire)
                dir_x = np.cross(dir_z, dir_up)
                dir_norm = np.linalg.norm(dir_x)
                dir_x = dir_x / dir_norm
                s = random.uniform(-traj_noise,traj_noise)             #行车轨迹的尺度大小
                scaling = dir_x * s
                points_tmp_height = basepoints[j] - scaling
                points_tmp_height[1] = points_tmp_height[1] + height
                points_tmp.append(points_tmp_height)
            points_tmp_height = basepoints[len(basepoints)-1] - scaling
            points_tmp_height[1] = points_tmp_height[1] + height
            points_tmp.append(points_tmp_height)          #添加最后一个点，有效性待验证
            points_tm = interpolate3d(points_tmp,sample_N-1)      #插值之后的轨迹  插值点数
            nn = int((sample_N-1)/kf_count)                                     #行车轨迹变成keyframe的频率
            keypoints = []
            for m in range(0, nn):
                keypoints.append(points_tm[kf_count * m])
                trajectory['frame_id'].append(x)
                x = x + 1
            pose_list = core_data_helper.gen_points_traj(keypoints)
            pose_lists.append(pose_list)
            trajectorys.append(trajectory)
    return pose_lists

def project_road(core_data,frame_id_list):
    if 'lines' not in core_data.keys():
        return
    if len(frame_id_list) == 0:
        return
    width = core_data['width']
    height = core_data['height']
    view_angle = core_data['view_angle']
    k = core_data_helper.cal_k(height, width, view_angle)
    frames = []
    for n in range(0, len(frame_id_list)):
        frames.append(core_data['frames'][frame_id_list[n]])
    line_count = len(core_data['lines'])
    for j in range(0, len(frames)):
        if 'line_list' not in frames[j]:
            frames[j]['line_list'] = []
            line_lists = frames[j]['line_list']
        for i in range(0, line_count):
            line_list = {}
            line = core_data['lines'][i]
            points = line['line_points']
            uvs, ids = core_data_helper.project_road3d(k, np.matrix(frames[j]['pose']), points, width, height)
            if ids:
                line_list['line_id'] = i
                line_list['line_pt_id']= ids
            else:
                line_list['line_id'] = []
                line_list['line_pt_id'] = []
            line_lists.append(line_list)
            if 'images' not in core_data['lines'][i]:
                core_data['lines'][i]['images'] = []
            image = {}
            image['frame_id'] = j
            image['uvs'] = uvs
            core_data['lines'][i]['images'].append(image)

def show_road(ax,core_data,linesample_FLAG):
    for j in range(0, len(core_data['lines'][0]['line_points']) - 1):
        ax.plot3D([core_data['lines'][0]['line_points'][j][0], core_data['lines'][0]['line_points'][j + 1][0]],
                  [core_data['lines'][0]['line_points'][j][1], core_data['lines'][0]['line_points'][j + 1][1]],
                  [core_data['lines'][0]['line_points'][j][2], core_data['lines'][0]['line_points'][j + 1][2]], 'y',
                  linewidth=3)                                                                 #显示主干道
    for i in range(1, len(core_data['lines'])):
        for j in range(0, len(core_data['lines'][i]['line_points']) - 1):
            ax.plot3D([core_data['lines'][i]['line_points'][j][0], core_data['lines'][i]['line_points'][j + 1][0]],
                      [core_data['lines'][i]['line_points'][j][1], core_data['lines'][i]['line_points'][j + 1][1]],
                      [core_data['lines'][i]['line_points'][j][2], core_data['lines'][i]['line_points'][j + 1][2]], 'w',
                      linewidth=3)                                                         #显示车道线
    if linesample_FLAG:
        for i in range(0, len(core_data['lines'])):
            for j in range(0,len(core_data['lines'][i]['line_points'])):
                ax.scatter3D(core_data['lines'][i]['line_points'][j][0], core_data['lines'][i]['line_points'][j][1],
                             core_data['lines'][i]['line_points'][j][2],c='b')


def road_image(ax, core_data, frame_id):
    for j in range(0,len(core_data['lines'])):
        if j == 0:
            # color1 = core_data_helper.get_rand_color()
            color1 = 'y'
        else:
            color1 = 'w'
        for n in range(0,len(core_data['lines'][j]['images'])):
            if frame_id == core_data['lines'][j]['images'][n]['frame_id']:
                uvs = core_data['lines'][j]['images'][n]['uvs']
                for i in range(0,len(uvs)-1):
                    ax.plot([uvs[i][0],uvs[i+1][0]],[uvs[i][1],uvs[i+1][1]],color=color1,linewidth=3)





