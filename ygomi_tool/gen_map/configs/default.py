def get_config():
    config={}
    #some switches
    # for output
    config['save_core_data'] = False  #save data
    config['show_core_data'] = True   #save the visualization files
    config['is_show'] = False  # 是否显示画图figure
    config['gen_sever_files'] = True  #generate server input files
    # for scene
    config['generate_road'] = True #generate road in the scene
    config['generate_ts'] = True #generate traffic sign in the scene
    config['generate_lm'] = True #generate landmark in the scene
    config['add_descriptor'] = True #add descriptor in the mps
    #for visualization
    config['camera_FLAG'] = True       #keyframe 是否连接起来
    config['linesample_FLAG'] = False   #车道线的采样点是否连接起来
    config['sample_numble']   = 51      #骨架线采样点数 数字加1,一个bug待以后修复
    #directory config
    config['working_dir'] = ''

    #scene config
    config['frame_count']=250        #只针对圆形轨迹
    config['box_count']= 5           #the number of boxes
    config['trafficsign_count'] = 5  #the number of traffic sign
    config['scene_width'] = 120      #the width of show scene,if in the real scene,it could be 500
    config['half_lane_count'] = 1    #主干道一侧车道个数
    config['lane_width'] = 4         #车道宽度
    config['lane_number'] = [0,1]    #车道编号
    config['traj_count'] = [1,1]     #每个车道生成的轨迹数量

    #trajectory config
    config['traj_noise'] = 0.3  #轨迹在车道内摆动的幅度
    config['kf_count'] = 2     #每隔几个采样点生成一个KF

    #camera config
    config['width'] = 1024
    config['height'] = 768
    config['view_angle'] = 60

    #input data
    config['road_bone'] = [[-100,0,50],[-70,0,50],[-50,0,50],[-35,0,50],[-20,0,50],[-10,0,50],[0,0,50],[10,0,49],[20,0,45.8],[30,0,40],[40,0,30],[45,0,21.7],    #主干道骨架线
                   [50,0,0],[50,0,-10],[50,0,-20],[50,0,-30],[50,0,-50],[50,0,-70],[50,0,-100]]
    #The actual road length
    # config['road_bone'] = [[-200,0,400],[-150,0,400],[-100,0,400],[-50,0,400],[0,0,400],[100,0,400],[200,0,400],[300,0,400],[310,0,399.5],[320,0,398],[330,0,395.4],[340,0,391.7],
    #                        [350,0,386.6],[360,0,380],[370,0,371.4],[380,0,360],[390,0,343.6],[400,0,300],[400,0,200],[400,0,100],[400,0,50],[400,0,0],[400,0,-100],[400,0,-200]]
    return config




