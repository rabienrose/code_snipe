import numpy as np
import matplotlib.pyplot as plt
import csv
import sys

markCount = 10
rootAddr = sys.argv[1]

x_slam= np.array([])
y_slam= np.array([])
with open(rootAddr + '/slam_pose_m.txt', newline='') as csvfile:
    spamreader = csv.reader(csvfile)
    for row in spamreader:
        x_slam = np.append(x_slam, float(row[1]))
        y_slam = np.append(y_slam, float(row[3]))


x_gps= np.array([])
y_gps= np.array([])
with open(rootAddr + '/gps.txt', newline='') as csvfile:
    spamreader = csv.reader(csvfile)
    for row in spamreader:
        items = row[0].split(' ')
        x_gps = np.append(x_gps, float(items[0]))
        y_gps = np.append(y_gps, float(items[1]))


avi_x = np.mean(x_gps)
avi_y = np.mean(y_gps)
x_gps = x_gps - avi_x
y_gps = y_gps - avi_y
x_slam = x_slam - avi_x
y_slam = y_slam - avi_y

line_gps, = plt.plot(x_gps, y_gps)
line_gps.set_label('GPS')
line_slam, = plt.plot(x_slam, y_slam)
line_slam.set_label('SLAM')
plt.legend()


frameCount = len(x_slam)
step = int(frameCount / markCount)
for i in range(0, frameCount-1):
    if i%step == 0:
        plt.plot([x_gps[i],x_slam[i]], [y_gps[i],y_slam[i]],'#000000')



plt.axes().set_aspect('equal')
plt.show()