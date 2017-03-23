#!/usr/local/bin/python3.5
# encoding=utf-8

import os
#mypath = "/Volumes/chamo/dataset/KITTI/00/image_0/"
#mypath = "/Volumes/chamo/dataset/iPhone6_GPS/GM/Red_truck_iPhone_10FA9042-A17F-44CB-AA80-0EC19A6CCE8F/JPG_and_GPS/cap_20160523152741/jpg_20160523152741/"
#mypath = "/Volumes/chamo/dataset/overpass_west_to_south/images/"
mypath = "/Volumes/chamo/dataset/LSD/LSD_room1/images/"
f = []
for root, dirs, files in os.walk(mypath):
	for name in files:
		name1 = name.lstrip("0")
		os.rename(mypath + name, mypath + name1)
