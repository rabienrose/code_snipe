#encoding=utf-8
import os
import matplotlib.pyplot as plt
import trafficsignfunction
import slamfunction
import roadgeofunction
import core_data_helper

def display_scene(core_data, frameid, scene_width):
    ax = core_data_helper.get_3d_ax(scene_width)
    slamfunction.show_landmark(ax, core_data)
    slamfunction.show_frameid_camera(ax, core_data, frameid, True)
    trafficsignfunction.show_trafficsign(ax, core_data, frameid)
    roadgeofunction.show_road(ax, core_data, False)
    plt.show()

def show_scene_all(core_data, save_addr, scene_width,camera_FLAG,linesample_FLAG):
    if not os.path.exists(save_addr):
        os.makedirs(save_addr)
    frame_count = len(core_data['frames'])
    for i in range(0,frame_count):
        frame_id = i
        ax = core_data_helper.get_3d_ax(scene_width)
        slamfunction.show_landmark(ax,core_data)
        slamfunction.show_frameid_camera(ax, core_data, frame_id,camera_FLAG)
        trafficsignfunction.show_trafficsign(ax, core_data, frame_id)
        roadgeofunction.show_road(ax,core_data,linesample_FLAG)
        address = save_addr + 'scene_' + str(frame_id) + '.png'
        plt.savefig(address,bbox_inches='tight', pad_inches=0)

def show_image_all(core_data, save_addr):
    if not os.path.exists(save_addr):
        os.makedirs(save_addr)
    frame_count = len(core_data['frames'])
    for i in range(0,frame_count):
        frame_id = i
        ax = core_data_helper.get_2d_ax()
        slamfunction.show_image(ax, core_data, frame_id)
        trafficsignfunction.trafficsignimage(ax, core_data, frame_id)
        roadgeofunction.road_image(ax, core_data, frame_id)
        address = save_addr + 'image_' + str(frame_id) + '.png'
        plt.savefig(address,bbox_inches='tight', pad_inches=0)

def generate_frame(core_data, pose_list):
    if 'frames' not in core_data.keys():
        core_data['frames'] =[]
    frames = core_data['frames']
    for i in range(0, len(pose_list)):
        frame ={}
        frame['id'] = len(frames)
        frame['pose'] = pose_list[i].tolist()
        frames.append(frame)

