import numpy as np
import matplotlib.pyplot as plt
import csv
import sys
import scipy.linalg
import math

rootAddr = sys.argv[1]
poseData = np.genfromtxt(rootAddr + '/slam_pose.txt',delimiter=' ')
poseList = []
for row in poseData:
    poseList.append(np.matrix(row[0:12].reshape(3,4)))

camCenter = []
for pose in poseList:
    Rcw = pose[0:3,0:3]
    tcw = pose[0:3,3]
    Rwc = np.transpose(Rcw)
    C = -np.dot(Rwc, tcw)
    uv=C[[2,0]]
    camCenter.append(uv)
    #plt.plot(uv[0], uv[1], 'ro')

gpsData = np.genfromtxt(rootAddr + '/gps.txt',delimiter=' ')
gpsUV =[]
for row in gpsData:
    uv =np.matrix([[row[0]],[row[1]]])
    gpsUV.append(uv)
    #plt.plot(uv[0], uv[1], 'ro')

lastItem = len(gpsUV)-1
dirGPS = gpsUV[lastItem] - gpsUV[0]
dirSLAM = camCenter[lastItem] - camCenter[0]
lenGPS = math.sqrt(dirGPS[0]*dirGPS[0] + dirGPS[1]*dirGPS[1])
lenSLAM = math.sqrt(dirSLAM[0]*dirSLAM[0] + dirSLAM[1]*dirSLAM[1])
offset =camCenter[0] - gpsUV[0]
theta = np.dot(dirGPS, dirSLAM)
#for i in range(0,len(gpsUV)):


# x_gps= np.array([])
# y_gps= np.array([])
# with open(rootAddr + '/gps.txt', newline='') as csvfile:
#     spamreader = csv.reader(csvfile)
#     for row in spamreader:
#         items = row[0].split(' ')
#         x_gps = np.append(x_gps, float(items[0]))
#         y_gps = np.append(y_gps, float(items[1]))
# plt.plot(x_gps, y_gps, 'bo')
plt.axes().set_aspect('equal')
plt.show()