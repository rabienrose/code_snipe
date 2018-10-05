import sys
import os
#---------------------config_page--------------------------


#*******************1.run_localization*****************************

parent_path=os.path.abspath(sys.argv[0]).split('/py_script')[0]

#------------------------input--------------------------------------
code_path = parent_path+'/code/core'
database_path = parent_path+'/data_center/input/db'
rtv_files = parent_path+'/data_center/input/rtv'
rosbag_files = ''
stop_rosbag = ''
bow_file = parent_path+'/code/core/algorithm_vehicle_localization/config/merged_two_lanes_voc.bin'

#------------------------output-------------------------------------
loc_result_path = parent_path+'/data_center/output/loc_result'
debug_kml_collection_path = parent_path+'/data_center/output/debug_kml_collection'
problem_list_collection_path = parent_path+'/data_center/output/problem_list_collection'

#-------------------------parameter_option--------------------------
ros_start_time = '0'
ros_total_time = '30000'
rtv_start_frame = '0'
rtv_total_frames = '30000'
comamnd_log_level = '1'


#*******************2.compare_result*****************************

kml_estimation_exe_path=parent_path+'/code/core/algorithm_vehicle_localization/examples/db_est_tool/build/kml_estimation'

track_2d_count_flag = "total 2d track count:"
track_3d_count_flag = "total 3d match count:"
mse_count_flag = "track drop by MSE:"
epi_count_flag = "track drop by epi:"
#------------------------input--------------------------------------
case_db_reference_path=parent_path+'/data_center/input/reference'
temp_folder=loc_result_path.split(loc_result_path.split('/')[-1])[0]+'temp_folder'

#------------------------output-------------------------------------
compare_resut_txt_path=parent_path+'/data_center/output/compare'


#*******************3.evaluate_result*****************************

#------------------------output-------------------------------------
evaluate_result_txt_path=parent_path+'/data_center/output/evaluate_output'

###############jenkins#################
jenkins_folder='/media/psf/Untitled/auto_loc_workspace/jenkins'
storage_transfer_folder='/media/psf/Untitled/auto_loc_workspace/jenkins/storage_transfer_folder'
show_task_folder_path='/media/psf/Untitled/auto_loc_workspace/task_folder'
