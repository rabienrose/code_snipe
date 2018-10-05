import math
import random
import subprocess

import matplotlib.pyplot as plt
import numpy as np

def calProjectErr(pose, mps, uvs, fx, fy,cx,cy):
    cul_error = 0
    for i in range(0, len(mps)):
        mp_norm = np.matrix(np.ones((4,1),np.float))
        mp_norm[0:3,0] = mps[i][0:3,0]
        pose_norm = np.matrix(np.identity(4,np.float))
        pose_norm[0:3,0:4] = pose[0:3,0:4]
        k=np.matrix(np.identity(4,np.float))
        k[0,0] = fx
        k[1,1] = fy
        k[0,2] = cx
        k[1,2] = cy
        posi_p = k*pose_norm*mp_norm
        u=posi_p[0,0]/posi_p[2,0]
        v=posi_p[1,0]/posi_p[2,0]
        err = math.sqrt(math.pow(uvs[i][0]- u,2)+math.pow(uvs[i][1]- v,2))
        cul_error= cul_error+err
    cul_error = cul_error/len(mps)
    return cul_error

def drawCamera(pose, ax, type, angle):
    rot = pose[0:3,0:3]
    t=pose[0:3,3]
    direction = rot.transpose()
    posi=-direction*t
    dir_x = direction[0:3,0]
    dir_y = direction[0:3,1]
    dir_z = direction[0:3,2]

    scale = 2
    width = scale*math.tan(angle)
    pt1=posi+dir_z*scale
    pt2=posi+dir_x*width+dir_z*scale
    pt3=posi-dir_x*width+dir_z*scale
    ax.plot([posi[0,0], pt2[0,0]], [posi[2,0], pt2[2,0]],'b')
    ax.plot([posi[0,0], pt3[0,0]], [posi[2,0], pt3[2,0]],'b')
    ax.plot([pt2[0,0], pt3[0,0]], [pt2[2,0], pt3[2,0]],'b')
    ax.plot(posi[0], posi[2], type)

def create_pose(theta_x, theta_y, theta_z, posi_x, posi_y, posi_z):
    pose =np.matrix([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0]])
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


init_pose_offset = [0,0,0,0,30,0]
width = 1024
height = 768
view_angle = 127
theta_x = 0
theta_y = 0
theta_z = 0
cen_x =0
cen_y=0
cen_z=0
view_angle_radian = view_angle/360*2*3.1415926
fx = width/2/math.tan(view_angle_radian/2)
fy = fx
cx=width/2
cy=height/2
k= [[fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1]]

pose = create_pose(theta_x, theta_y, theta_z, cen_x, cen_y, cen_z)
pose_inv = np.linalg.inv(pose)
random.seed(0)
mp_count = 100
max_depth = 10
pts=[]
depth =[]
for i in range(0,mp_count):
    depth_t = random.uniform(1, max_depth)
    depth.append(depth_t)
    u_t = random.uniform(width/2, width)
    v_t = random.uniform(0, height)
    pts.append([u_t, v_t])

xs=[]
ys=[]
zs=[]
for i in range(0, mp_count):
    x = (pts[i][0]-cx)*depth[i]/fx
    y = (pts[i][1]-cy)*depth[i]/fy
    #d_n = random.uniform(-1, 1)
    d_n = 0
    p_local = np.matrix([[x+d_n],[y+d_n],[depth[i]],[1]])
    p=pose_inv * p_local
    xs.append(p[0, 0])
    ys.append(p[1, 0])
    zs.append(p[2, 0])

fig = plt.figure()
ax = fig.gca()
ax.plot(xs, zs,'k.')
drawCamera(pose, ax, 'ro', view_angle_radian/2)

f = open('pair.csv', 'w')
for i in range(0, mp_count):
    record = '%f,%f,%f,%f,%f,%f\n'% (pts[i][0], pts[i][1], depth[i], xs[i], ys[i], zs[i])
    f.write(record)
f.close()

pose_noise = create_pose(theta_x+init_pose_offset[3], theta_y+init_pose_offset[4], theta_z+init_pose_offset[5],
                         cen_x+init_pose_offset[0], cen_y+init_pose_offset[1], cen_z+init_pose_offset[2])
drawCamera(pose_noise, ax, 'r*', view_angle_radian/2)
f = open('pose.csv', 'w')
for i in range(0,4):
    row = '%f,%f,%f,%f\n'%(pose[i, 0], pose[i, 1], pose[i, 2], pose[i, 3])
    f.write(row)
for i in range(0,4):
    row = '%f,%f,%f,%f\n'%(pose_noise[i, 0], pose_noise[i, 1], pose_noise[i, 2], pose_noise[i, 3])
    f.write(row)
f.close()

f = open('k.csv', 'w')
row = '%f,%f,%f,%f\n'%(fx, fy, cx, cy)
f.write(row)
f.close()

mps = []
for i in range(0, len(xs)):
    mp = np.matrix(np.zeros((3,1),np.float))
    mp[0,0] = xs[i]
    mp[1,0] = ys[i]
    mp[2,0] = zs[i]
    mps.append(mp)
err_before = calProjectErr(pose_noise, mps, pts, fx, fy,cx,cy)
print('err before: ' + str(err_before))
output_addr = "/Volumes/chamo/working/test_case_rf/script/gen_true_data"
subprocess.run(["/Volumes/chamo/working/slam2_repo/core/algorithm/examples/module_test/pose_optimise/test_pose_opt", output_addr])


f = open('pose_opt.csv', 'r')
pose_opt =np.matrix([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0]])
for i in range(0,3):
    row = f.readline()
    strList = row.split(',')
    for j in range(0,4):
        pose_opt[i,j]=float(strList[j])
f.close()

err_after = calProjectErr(pose_opt, mps, pts, fx, fy,cx,cy)
print('err after: ' + str(err_after))
drawCamera(pose_opt, ax, 'y*', view_angle_radian/2)

ax.set_aspect('equal')
plt.xlabel('x(m)')
plt.ylabel('y(m)')
plt.title('test init pose')
plt.figtext(0.3,0.05,'err before: ' + str(err_before), color='r')
plt.figtext(0.3,0.005,'err after: ' + str(err_after), color='r')
plt.show()