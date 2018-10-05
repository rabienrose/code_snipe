import os
from os import listdir
from os.path import isfile, join
import os.path
import shutil

import subprocess
def rtv2imges(rtv_addr, image_addr, gps_addr):
    rtv_extractor_addr = '/media/psf/chamo/working/repo_master/framework/device/rtv-extractor/dist/x64/bin/rtv-extractor'
    onlyfiles = [f for f in listdir(rtv_addr) if isfile(join(rtv_addr, f))]
    for i in range(0, len(onlyfiles)):
        if onlyfiles[i] == '.DS_Store':
            continue
        base_name = os.path.splitext(onlyfiles[i])[0]
        if not os.path.exists(image_addr+base_name):
            os.makedirs(image_addr+base_name)
        bashCommand = rtv_extractor_addr+' -f '+rtv_addr+onlyfiles[i]+' -d '+ image_addr+base_name
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        print(error)
        shutil.copy(image_addr+base_name+'/gps.txt', gps_addr+base_name+'.txt')

def convert_gps2kml(file_name, out_file):
    f = open(file_name)
    f_out = open(out_file, 'w')
    lines = f.readlines()
    header='<?xml version = "1.0" encoding = "UTF-8"?><kml xmlns="http://earth.google.com/kml/2.2"><Placemark><LineString><coordinates>\n'
    footer='</coordinates></LineString><Style><LineStyle><color>#ffff00ff</color><width>5</width></LineStyle></Style></Placemark></kml>'
    f_out.write(header)
    for kml_line_ind in range(0, len(lines)):
        line = lines[kml_line_ind]
        gps_data = line.split(',')
        gps_str = gps_data[3]+','+gps_data[2]+','+gps_data[4]
        f_out.write(gps_str)
    f_out.write(footer)
    f.closed
    f_out.closed

def txt2kml(gps_addr, kml_addr):
    onlyfiles = [f for f in listdir(gps_addr) if isfile(join(gps_addr, f))]
    for i in range(0, len(onlyfiles)):
        if onlyfiles[i] == '.DS_Store':
            continue
        base_name = os.path.splitext(onlyfiles[i])[0]
        convert_gps2kml(gps_addr+onlyfiles[i], kml_addr+base_name+'.kml')

def gps2rel(gps_addr, rel_addr):
    gps_converter_addr = '/media/psf/chamo/working/repo_master/core/tools/others/gpsTool/build/gpsTime2RelTool'
    onlyfiles = [f for f in listdir(gps_addr) if isfile(join(gps_addr, f))]
    for i in range(0, len(onlyfiles)):
        if onlyfiles[i] == '.DS_Store':
            continue
        bashCommand = gps_converter_addr + ' ' + gps_addr + onlyfiles[i] + ' ' + rel_addr + onlyfiles[i]
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        print(bashCommand)

root = '/media/psf/chamo/dataset/gm_new/'
#root = '/Volumes/chamo/dataset/gm_new/'
rtv_addr = root +'rtv/'
image_addr = root +'images/'
gps_addr = root +'gps/'
kml_addr = root +'kml/'
rel_addr = root +'ref/'

#rtv2imges(rtv_addr, image_addr, gps_addr)
#txt2kml(gps_addr, kml_addr)
gps2rel(gps_addr, rel_addr)