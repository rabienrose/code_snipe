#encoding=utf-8
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import random
import json

def axis_equal_3d(ax):
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)

def create_pose(posi_x, posi_y, posi_z, theta_x, theta_y, theta_z):
    pose =np.matrix(np.identity(4, np.float))
    theta_x =theta_x/360*2*3.1415926
    theta_y =theta_y/360*2*3.1415926
    theta_z =theta_z/360*2*3.1415926
    rx =np.matrix([[1, 0, 0],
                 [0, math.cos(theta_x), -math.sin(theta_x)],
                 [0, math.sin(theta_x), math.cos(theta_x)]])

    ry =np.matrix([[math.cos(theta_y), 0, math.sin(theta_y)],
                 [0, 1, 0],
                 [-math.sin(theta_y), 0, math.cos(theta_y)]])

    rz =np.matrix([[math.cos(theta_z), -math.sin(theta_z), 0],
                 [math.sin(theta_z), math.cos(theta_z), 0],
                 [0, 0, 1]])

    direction= rz*ry*rx
    rot = direction.transpose()
    position = np.matrix([[posi_x],[posi_y],[posi_z]])
    trans = -rot*position
    pose[0:3, 0:3] = rot
    pose[0:3, 3] = trans
    return pose

def gen_circle_traj(radius, frame_count):
    dir_up = np.array([0, 1, 0])
    theta_x = 0
    y = 0
    pose_list = []
    for i in range(0,frame_count):
        angle = i*(360/frame_count)
        angle = angle*2*math.pi/360
        dir_r = np.array([radius * math.cos(angle), 0, radius * math.sin(angle)])   #每个相机的位置向量
        dir_t = np.cross(dir_r,dir_up)                                              #法向量
        L1 = math.sqrt(np.square(abs(dir_t[2])) + np.square(abs(dir_t[0])) + np.square(abs(dir_t[1])))
        theta_y = 360*np.arctan2(dir_t[0], dir_t[2])/(2*math.pi)
        theta_z = 360*math.asin(dir_t[1]/L1)/(2*math.pi)
        z = radius * math.sin(angle)
        x = radius * math.cos(angle)
        pose = create_pose(x,y,z,theta_x,theta_y,theta_z)    #只旋转了theta_y方向
        pose_list.append(pose)
    return pose_list

#return the projected u,v points
def project_3d(k, pose, points, width, height):
    pose_tmp = k * pose
    uvs = []
    ids =[]
    for i in range(0, len(points)):
        uv_tmp = np.matrix(np.zeros((1, 2), np.float))
        mp_tmp = np.matrix(np.ones((4, 1), np.float))
        mp_tmp[0, 0] = points[i][0]
        mp_tmp[1, 0] = points[i][1]
        mp_tmp[2, 0] = points[i][2]
        imgtmp = pose_tmp * mp_tmp
        if imgtmp[2, 0] < 1:
            continue
        uv_tmp[0,0] = imgtmp[0, 0] / imgtmp[2, 0]
        uv_tmp[0,1] = imgtmp[1, 0] / imgtmp[2, 0]
        if uv_tmp[0,0] > 1 and uv_tmp[0,0] < width - 1 and uv_tmp[0,1] > 1 and uv_tmp[0,1] < height - 1:
            ids.append(i)
            uvs.extend(uv_tmp.tolist())
    return uvs, ids
#return the projected u,v points
def project_road3d(k, pose, points, width, height):
    pose_tmp = k * pose
    uvs = []
    ids =[]
    for i in range(0, len(points)):
        uv_tmp = np.matrix(np.zeros((1, 2), np.float))
        mp_tmp = np.matrix(np.ones((4, 1), np.float))
        mp_tmp[0, 0] = points[i][0]
        mp_tmp[1, 0] = points[i][1]
        mp_tmp[2, 0] = points[i][2]
        imgtmp = pose_tmp * mp_tmp
        if imgtmp[2, 0] < 1:
            continue
        uv_tmp[0,0] = imgtmp[0, 0] / imgtmp[2, 0]
        uv_tmp[0,1] = imgtmp[1, 0] / imgtmp[2, 0]
        if uv_tmp[0,0] > 1 and uv_tmp[0,0] < width - 1 and uv_tmp[0,1] > height*0.1 and uv_tmp[0,1] < height*0.6:
            ids.append(i)
            uvs.extend(uv_tmp.tolist())
    return uvs, ids

#return the transformed 3d points
def transform_point3d(points, trans_mat):
    trans_tmp = trans_mat.I
    mps = []
    for i in range(0,len(points)):
        mp_tmp = np.matrix(np.ones((4, 1), np.float))
        mp_tmp[0, 0] = points[i][0]
        mp_tmp[1, 0] = points[i][1]
        mp_tmp[2, 0] = points[i][2]
        point_tmp = trans_tmp * mp_tmp
        mps.append([point_tmp[0,0],point_tmp[1,0],point_tmp[2,0]])
    return mps


def cal_k(height, width, view_angle):
    view_angle_radian = view_angle / 360.0 * 2 * 3.1415926
    fx = width / 2 / math.tan(view_angle_radian / 2)
    fy = fx
    cx = width / 2
    cy = height / 2
    k = np.matrix([[fx, 0, cx, 0], [0, fy, cy, 0], [0, 0, 1, 0]])
    return k

def get_k(core_data):
    width = core_data['width']
    height = core_data['height']
    view_angle = core_data['view_angle']
    k = cal_k(height, width, view_angle)
    return k

def save_result(core_data, file_name):
    f = open(file_name, 'w')
    f.write(json.dumps(core_data))
    f.close()

def read_result(file_name):
    core_data={}
    return core_data

def get_2d_ax():
    fig = plt.figure('fig1')
    fig.clear()
    ax = fig.gca()
    ax.set_axis_bgcolor('k')  # 设置画布颜色
    ax.set_xlim(-120, 120)
    ax.set_ylim(-120, 120)
    plt.xticks([])
    plt.yticks([])
    plt.axis([0, 1024, 0, 768])
    return ax

def get_3d_ax(scene_width):
    fig = plt.figure('fig', figsize=(6, 6))
    fig.clear()
    ax = Axes3D(fig)
    ax.set_zlabel('Z')
    ax.set_ylabel('Y')
    ax.set_xlabel('X')
    ax.view_init(0, -90)
    ax.set_xlim(-scene_width, scene_width)
    ax.set_ylim(-scene_width, scene_width)
    ax.set_zlim(-scene_width, scene_width)
    ax.grid(False)
    ax.set_axis_bgcolor('k')  # 设置画布颜色
    return ax

def get_rand_color():
    r = int(random.uniform(0, 255))
    g = int(random.uniform(0, 255))
    b = int(random.uniform(0, 255))
    return '#%0.2X%0.2X%0.2X'%(r,g,b)

def get_pose_list(core_data, frame_id_list):
    pose_list = []
    for i in range(0, len(frame_id_list)):
        pose_list.append(np.matrix(core_data['frames'][frame_id_list[i]]['pose']))
    return pose_list

def gen_points_traj(points_list):
    pose_list = []
    dir_up = np.array([0, 1, 0])
    theta1 = 20*2*3.1415/360       #相机向一个方向倾斜的角度
    for i in range(0,len(points_list)-1):
        pose = np.matrix(np.identity(4, np.float))
        dire = [points_list[i+1][0]-points_list[i][0],points_list[i+1][1]-points_list[i][1],points_list[i+1][2]-points_list[i][2]]
        dir_z = np.array(dire)
        xx = math.sqrt(np.square(dire[0])+np.square(dire[2]))
        theta = math.atan2(dire[1],xx)
        s = math.tan(theta+theta1)*xx - dire[1]
        dir_z[1] = dir_z[1] + s
        dir_norm = np.linalg.norm(dir_z)
        dir_z = dir_z / dir_norm
        # dir_x = np.cross(dir_z,dir_up)     #交换位置?
        dir_x = np.cross(dir_up, dir_z)
        dir_y = np.cross(dir_z,dir_x)
        dir_norm = np.linalg.norm(dir_x)                 #归一化很重要
        dir_x = dir_x / dir_norm
        dir_norm = np.linalg.norm(dir_y)
        dir_y = dir_y / dir_norm

        #dir_z = dir_z * pose_change
        if i == 0:
            trans = [points_list[0][0],points_list[0][1],points_list[0][2]]
            trans = np.matrix(trans)
            pose[0, 0:3] = np.matrix(dir_x)
            pose[1, 0:3] = np.matrix(dir_y)
            pose[2, 0:3] = np.matrix(dir_z)
            rot = pose[0:3, 0:3].transpose()
            trans = rot.I * -trans.transpose()
            pose[0:3, 3] = trans
            pose_list.append(pose)
            trans = [points_list[1][0], points_list[1][1], points_list[1][2]]
            trans = np.matrix(trans)
            trans = rot.I * -trans.transpose()
            pose[0:3, 3] = trans
            pose_list.append(pose)
        else:
            trans = [points_list[i + 1][0], points_list[i + 1][1], points_list[i + 1][2]]
            trans = np.matrix(trans)
            pose[0, 0:3] = np.matrix(dir_x)
            pose[1, 0:3] = np.matrix(dir_y)
            pose[2, 0:3] = np.matrix(dir_z)
            rot = pose[0:3, 0:3].transpose()
            trans = rot.I * -trans.transpose()
            pose[0:3, 3] = trans
            pose_list.append(pose)
    return pose_list

def changepose(thetax,thetay,thetaz):
    theta_x = thetax / 360 * 2 * 3.1415926
    theta_y = thetay / 360 * 2 * 3.1415926
    theta_z = thetaz / 360 * 2 * 3.1415926
    rx = np.matrix([[1, 0, 0],
                    [0, math.cos(theta_x), -math.sin(theta_x)],
                    [0, math.sin(theta_x), math.cos(theta_x)]])

    ry = np.matrix([[math.cos(theta_y), 0, math.sin(theta_y)],
                    [0, 1, 0],
                    [-math.sin(theta_y), 0, math.cos(theta_y)]])

    rz = np.matrix([[math.cos(theta_z), -math.sin(theta_z), 0],
                    [math.sin(theta_z), math.cos(theta_z), 0],
                    [0, 0, 1]])

    rot = rz * ry * rx
    rot = rot.transpose()
    return rot

def checkr(pose):
     r = pose[0:3, 0:3]
     r1 = r.I
     Y = (np.mat([0,1,0])).transpose()
     y = r1 * Y
     return y