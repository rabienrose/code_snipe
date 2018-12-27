import matplotlib.pyplot as plt
import csv

ax=[]
ay=[]
az=[]
gx=[]
gy=[]
gz=[]
time=[]
with open('/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/11_19_office_garage/imu.txt') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    first_time=0
    for row in csv_reader:
        if line_count==0:
            first_time=float(row[1])
        time.append(float(row[1])-first_time)
        gx.append(float(row[2]))
        gy.append(float(row[3]))
        gz.append(float(row[4]))
        ax.append(float(row[5]))
        ay.append(float(row[6]))
        az.append(float(row[7]))
        line_count=line_count+1
plt.subplot(611)
plt.plot(time,ax)
plt.subplot(612)
plt.plot(time,ay)
plt.subplot(613)
plt.plot(time,az)
plt.subplot(614)
plt.plot(time,gx)
plt.subplot(615)
plt.plot(time,gy)
plt.subplot(616)
plt.plot(time,gz)
plt.show()
