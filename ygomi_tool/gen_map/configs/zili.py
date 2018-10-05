#encoding=utf-8
import configs.default

def get_config():
    config=configs.default.get_config()
    #some switches
    config['save_core_data'] = True
    config['show_core_data'] = False
    config['generate_road'] = True
    config['generate_ts'] = False
    config['generate_lm'] = True
    config['camera_FLAG'] = True       #keyframe 是否连接起来
    config['linesample_FLAG'] = False   #车道线的采样点是否连接起来
    config['is_show'] = True         #是否显示画图figure

    #directory config
    config['working_dir'] = '/Volumes/chamo/dataset/syn_data/output/'

    #input data
    config['road_bone'] = [[-100,0,50],[-70,0,50],[-50,0,50],[-35,0,50],[-20,0,50],[-10,0,50],[0,0,50],[10,0,49],[20,0,45.8],[30,0,40],[40,0,30],[45,0,21.7],
                   [50,0,0],[50,0,-10],[50,0,-20],[50,0,-30],[50,0,-50],[50,0,-70],[50,0,-100]]
    return config




