#encoding=utf-8
import core_data_obj
import core_data_helper
import slamfunction
import trafficsignfunction
import roadgeofunction
import configs.nina
import sortnumber

config = configs.nina.get_config()
core_data={}
core_data['width'] = config['width']
core_data['height'] = config['height']
core_data['view_angle'] = config['view_angle']
config['road_bone'] = roadgeofunction.interpolate3d(config['road_bone'],config['sample_numble'])

#generate scene objects
if config['generate_lm']:
    slamfunction.generate_landmark(core_data, config['box_count'],config['road_bone'],config['sample_numble']-1,config['lane_width'])
if config['add_descriptor']:
    slamfunction.add_descriptor(core_data,config['working_dir'])
if config['generate_ts']:
    trafficsignfunction.generate_ts(core_data, config['trafficsign_count'],config['road_bone'],config['sample_numble']-1,config['lane_width'])
if config['generate_road']:
    roadgeofunction.generate_road(core_data, config['road_bone'],config['half_lane_count'],config['lane_width'])

#generate vehicle trajectory
if not config['generate_road']:
    pose_list = core_data_helper.gen_circle_traj(config['scene_width-10'], config['frame_count'])
    core_data_obj.generate_frame(core_data, pose_list)
else:
    pose_lists = roadgeofunction.generate_veh_traj(core_data, config['lane_number'], config['traj_count'], config['traj_noise'], config['kf_count'],config['sample_numble'])
    for i in range(0, len(pose_lists)):
        pose_list = pose_lists[i]
        core_data_obj.generate_frame(core_data, pose_list)

#projection
if config['generate_lm']:
    slamfunction.project_landmark(core_data, range(0, len(core_data['frames'])))
if config['generate_ts']:
    trafficsignfunction.project_ts(core_data, range(0, len(core_data['frames'])))
if config['generate_road']:
    roadgeofunction.project_road(core_data,range(0,len(core_data['frames'])))




#visualization
if config['save_core_data']:
    core_data_helper.save_result(core_data, config['working_dir'] + 'output.txt')
if config['show_core_data']:
    core_data_obj.show_scene_all(core_data, config['working_dir'] + 'scene/', config['scene_width'],config['camera_FLAG'],config['linesample_FLAG'])
    core_data_obj.show_image_all(core_data, config['working_dir'] + 'image/')

#rearrangement
if config['gen_sever_files']:
    sortnumber.sort_number(core_data, config['working_dir']+'output/')

if config['is_show']:
    core_data_obj.display_scene(core_data,1 ,config['scene_width'])