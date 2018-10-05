# -*- coding: utf-8 -*-
import os
import string
import commands  
import variables
import shutil
def kml_estimation_func(kml_estimation_exe_path,output_folder_path,single_db_path):
    with open(output_folder_path + '/kml_estimation_result.txt', 'w') as kml_estimation_txt:
        kml_estimation_txt.write('case              db              score_value'+'\n')
    case_name_list=os.listdir(single_db_path)
    for case_name in case_name_list:
        case_name_path=os.path.join(single_db_path,case_name)
        for root, dirs, files in os.walk(case_name_path):
            for csv_file in files:
                if 'locsummary.csv' in csv_file:
                    csv_file_path=os.path.join(root,csv_file)
                    try:
                        cmd=kml_estimation_exe_path+' '+csv_file_path+' chamo'
                        value=commands.getstatusoutput(cmd)
			value1=float(value[1])
			value2=("%.3f" % value1)
                        loc_case_output=output_folder_path + '/' +case_name+'_bad.kml'
                        shutil.copyfile('chamo_bad.kml', loc_case_output)
                        with open(output_folder_path + '/kml_estimation_result.txt', 'a+') as kml_estimation_txt:
                            kml_estimation_txt.write(case_name.split('_T_')[1]+' '+ single_db_path.split('/')[-1]+' '+str(value2)+ '\n')
		    except Exception as error:
                        print(error)
                        pass
                    break

def kml_estimation_func_by_diff_db(kml_estimation_exe_path,loc_result_path,output_folder_path):
    for single_db in os.listdir(loc_result_path):
	single_db_path=os.path.join(loc_result_path,single_db)
	kml_estimation_func(kml_estimation_exe_path,output_folder_path,single_db_path)


'''
kml_estimation_exe_path='/media/psf/Untitled/auto_loc_workspace/chamo/code/core/algorithm_vehicle_localization/examples/db_est_tool/build/kml_estimation'
loc_result_path='/media/psf/Untitled/auto_loc_workspace/task_folder/chamo/all_result_18-04-23_06:04:10_PM/loc_result'
output_folder_path='/media/psf/Untitled/auto_loc_workspace/hyf_test'

kml_estimation_func_by_diff_db(kml_estimation_exe_path,loc_result_path,output_folder_path)
'''
