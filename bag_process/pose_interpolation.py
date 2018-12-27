# -*- coding: utf-8 -*-
"""
Created on Sat Sep 29 10:16:05 2018

@author: albert.chang
"""
###input:1 traj.txt, created by LeGo_LOAM
#        2 image_time.txt, created by extract_img
#        3 calib.yml, created by cam_laser_calib, remove all annotation in yaml file when using  
#  output: image_pose_new.txt


import numpy as np
#from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
# import roslib
# import rosbag
# import rospy
# import cv2
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from cv_bridge import CvBridgeError
import yaml
import numpy.linalg as lg
import math
import sys

def rpy_to_quat(roll,pitch,yaw):
    # compute quaternion from XYZ fixed Euler angles
    # RPY = gamma, beta, alpha
    #     = phi, theta, psi
    cg = math.cos(roll/2)
    sg = math.sin(roll/2)
    cb = math.cos(pitch/2)
    sb = math.sin(pitch/2)
    ca = math.cos(yaw/2)
    sa = math.sin(yaw/2)

    q = []
    q.append(cg * cb * ca + sg * sb * sa)
    q.append(sg * cb * ca - cg * sb * sa)
    q.append(cg * sb * ca + sg * cb * sa)
    q.append(cg * cb * sa - sg * sb * ca)

    return q

def quat_to_rpy(qx,qy,qz,qw):
    rpy = []
    rpy.append(math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy)))
    rpy.append(math.asin(2*(qw*qy-qz*qx)))
    rpy.append(math.atan2(2*(qw*qz+qx*qy),1-2*(qx*qx+qy*qy)))

    return rpy

pose_data=[]
img_data=[]

##get pose information
with open(sys.argv[1]+'/traj.txt', 'r') as txtData:
    lines=txtData.readlines()
    for line in lines:
        lineData=line.strip().split(' ')  
        pose_data.append(lineData)
time  = np.zeros(len(pose_data))
x     = np.zeros(len(pose_data))
y     = np.zeros(len(pose_data))
z     = np.zeros(len(pose_data))
roll  = np.zeros(len(pose_data))
pitch = np.zeros(len(pose_data))
yaw   = np.zeros(len(pose_data))
qx    = np.zeros(len(pose_data))
qy    = np.zeros(len(pose_data))
qz    = np.zeros(len(pose_data))
qw    = np.zeros(len(pose_data))

for i in range(len(pose_data)):
    time[i]  = float(pose_data[i][1])-1538000000.0 ##for precision
    x[i]     = float(pose_data[i][2])
    y[i]     = float(pose_data[i][3])
    z[i]     = float(pose_data[i][4])
    roll[i]  = float(pose_data[i][5])
    pitch[i] = float(pose_data[i][6])
    yaw[i]   = float(pose_data[i][7])
    q        = rpy_to_quat(roll[i],pitch[i],yaw[i])
    qx[i]    = q[1]
    qy[i]    = q[2]
    qz[i]    = q[3]
    qw[i]    = q[0]

##get image and related time
# bridge = CvBridge()
# img_count = -1
# with rosbag.Bag('/media/albert/新加卷/2018-09-27/2018-09-27-17-07-59_clockwise.bag', 'r') as bag:
#     for topic, msg, t in bag.read_messages():
#         if topic == "/camera/image_color/compressed":  # 图像的topic；
#             try:
#                 cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
#             except CvBridgeError as e:
#                 print
#                 e
#             ++img_count
#             imgnamestr = "%d" % img_count
#             timesec = msg.header.stamp.to_sec()
#
#             # %.6f表示小数点后带有6位，可根据精确度需要修改；
#             image_name = "img/image" + imgnamestr + ".jpg"  # 图像命名：时间戳.jpg
#             cv2.imwrite(image_name, cv_image)  # 保存；
#             if timesec < (time[0] + 1538000000.0) or timesec > (time[-1] + 1538000000.0): #ignore image which is not in the span of pose
#                 continue
#             img_data.append([img_count, timesec])
camera_count = 0
while(len(sys.argv) - 2 - camera_count > 0):

    ##get image time
    with open(sys.argv[1]+'/camera_'+sys.argv[camera_count+2]+'_image_time.txt', 'r') as img_txt:
         lines=img_txt.readlines()
         for line in lines:
             lineData=line.strip().split('  ')
             if float(lineData[1]) < (time[0] + 1538000000.0) or float(lineData[1]) > (time[-1] + 1538000000.0):
                 continue
             img_data.append(lineData)

    img_time  = np.zeros(len(img_data))
    img_num   = np.zeros(len(img_data))
    for i in range(len(img_data)):
        img_time[i]  = float(img_data[i][1])-1538000000.0 ##for precision
        img_num[i]   = int(img_data[i][0])

    ##interpolation for x y z roll pitch yaw
    fx=interp1d(time,x,kind='cubic')
    image_x=fx(img_time)
    fy=interp1d(time,y,kind='cubic')
    image_y=fy(img_time)
    fz=interp1d(time,z,kind='cubic')
    image_z=fz(img_time)
    froll=interp1d(time,roll,kind='cubic')
    image_roll=froll(img_time)
    fpitch=interp1d(time,pitch,kind='cubic')
    image_pitch=fpitch(img_time)
    fyaw=interp1d(time,yaw,kind='cubic')
    image_yaw=fyaw(img_time)
    fqx=interp1d(time,qx,kind='cubic')
    image_qx=fqx(img_time)
    fqy=interp1d(time,qy,kind='cubic')
    image_qy=fqy(img_time)
    fqz=interp1d(time,qz,kind='cubic')
    image_qz=fqz(img_time)
    fqw=interp1d(time,qw,kind='cubic')
    image_qw=fqw(img_time)
    
    ##read yaml file to get extrinsic parameters
    with open(sys.argv[1]+"/calib_camera_"+sys.argv[camera_count+2]+".yml", "r") as yaml_file:
        yaml_obj = yaml.load(yaml_file.read())
        broker = yaml_obj["CameraExtrinsicMat"]
        trans_array = np.array(broker['data'])
        print(trans_array)

    Rotate = np.zeros((3,3))
    Translate = np.zeros((3,1))
    for i in range(3):
        for j in range(4):
            if j == 3 :
                Translate[i][0] = -trans_array[i*4 + j]
                continue
            Rotate[i][j] = trans_array[i*4 + j]

    R0 = np.array([[0,0,1],[1,0,0],[0,1,0]]) #loam->lidar

    #translate from camera to local
    R_I = lg.inv(Rotate.dot(R0))
    print(R_I)
    T_ = R_I.dot(Translate)
    print(T_)
    RT_ = np.hstack([R_I,T_])
    array_tmp = np.array([0, 0, 0, 1])
    RT = np.vstack([RT_,array_tmp])

    rad2degree = 180/math.pi

    with open(sys.argv[1]+'/camera_'+sys.argv[camera_count+2]+'_image_pose.txt', 'w') as img_pose:
        #img_pose.write("Fomat: No. of image / time / x / y / z / roll / pitch / yaw \n")
        img_pose.write("Fomat: No. of image /image_name/ time / rotation matrix listed by row \n")
        for i in range(len(img_time)):
            # linewrite= "{0:10s}{1:16.5f} {2:16.6e} {3:16.6e} {4:16.6e} {5:16.6e} {6:16.6e} {7:16.6e}\n".format(str(img_num[i]),
            #                                                              img_time[i] + 1538000000.0, image_x[i], image_y[i],
            #                                                              image_z[i],image_roll[i], image_pitch[i], image_yaw[i])
            #translate from local to global
            yaw_angle   = image_yaw[i]
            roll_angle  = image_roll[i]
            pitch_angle = image_pitch[i]
            
            # q0 = qw[i]
            # q1 = qx[i]
            # q2 = qy[i]
            # q3 = qz[i]
            # rpy = quat_to_rpy(q0,q1,q2,q3)
            
            ##roll_angle  = rpy[0]
            ##pitch_angle = rpy[1]
            ##yaw_angle   = rpy[2]
            
            Rz = np.array([[math.cos(yaw_angle), -math.sin(yaw_angle), 0],
                          [math.sin(yaw_angle),  math.cos(yaw_angle), 0],
                          [          0,            0, 1]])
            Rx = np.array([[1,           0,              0],
                          [0, math.cos(roll_angle), -math.sin(roll_angle)],
                          [0, math.sin(roll_angle),  math.cos(roll_angle)]])
            Ry = np.array([[ math.cos(pitch_angle), 0, math.sin(pitch_angle)],
                          [ 0,             1,             0],
                          [-math.sin(pitch_angle), 0, math.cos(pitch_angle)]])
            R_pose = Ry.dot(Rx.dot(Rz))
            T_pose = np.array([[image_x[i], image_y[i],image_z[i]]])
            RT_pose_ = np.hstack([R_pose,T_pose.T])
            RT_pose = np.vstack([RT_pose_, array_tmp])

            RT_total = RT_pose.dot(RT)
            linewrite= "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}\n".format(img_num[i],"img_"+str(int(img_num[i]))+".jpg", img_time[i] + 1538000000.0,
                                                                                                   RT_total[0][0],RT_total[0][1],RT_total[0][2],RT_total[0][3],
                                                                                                   RT_total[1][0],RT_total[1][1],RT_total[1][2],RT_total[1][3],
                                                                                                   RT_total[2][0],RT_total[2][1],RT_total[2][2],RT_total[2][3])

            # thetax = math.atan2(RT_pose[0][2],RT_pose[2][2]) * rad2degree
            # thetay = math.atan2(-RT_pose[1][2],math.sqrt(RT_pose[2][2]**2+RT_pose[0][2]**2)) * rad2degree
            # thetaz = math.atan2(RT_pose[1][0],RT_pose[1][1]) * rad2degree
            #
            # print("Euler Angle: ",thetax, thetay, thetaz)

            img_pose.write(linewrite)

    camera_count = camera_count + 1


##plot the interpolation result
# plt.plot(time,x,'r',label='origin')
# plt.plot(img_time,image_x,'b--',label='interpolation')
# plt.legend()
# plt.show()
