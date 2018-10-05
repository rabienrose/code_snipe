# -*- coding: utf-8 -*-
import variables
import run_loc_cases
import kml_estimation
import compare
import evaluate50
import log_handle
import os
import shutil
import sys
from datetime import datetime

if __name__ == '__main__':

    # -----judge input null----------------
    if not os.listdir(variables.database_path):
        print 'database_path:inuput null!!!'

    if not os.listdir(variables.rtv_files):
        print 'rtv_files:inuput null!!!'

    if not os.listdir(variables.case_db_reference_path):
        print 'case_db_reference_path:inuput null!!!'

    # ---------judge output folder null------------
    if not os.path.exists(os.path.dirname(variables.loc_result_path)):
        os.mkdir(os.path.dirname(variables.loc_result_path))

    if not os.path.exists(variables.loc_result_path):
        os.mkdir(variables.loc_result_path)

    if not os.path.exists(variables.compare_resut_txt_path):
        os.mkdir(variables.compare_resut_txt_path)

    if not os.path.exists(variables.evaluate_result_txt_path):
        os.mkdir(variables.evaluate_result_txt_path)

    if not os.path.exists(variables.debug_kml_collection_path):
        os.mkdir(variables.debug_kml_collection_path)

    if not os.path.exists(variables.problem_list_collection_path):
        os.mkdir(variables.problem_list_collection_path)

    # ------------judge output file exist-----------
    if os.listdir(variables.loc_result_path):
        shutil.rmtree(variables.loc_result_path)
        os.mkdir(variables.loc_result_path)

    if os.listdir(variables.compare_resut_txt_path):
        shutil.rmtree(variables.compare_resut_txt_path)
        os.mkdir(variables.compare_resut_txt_path)

    if os.listdir(variables.evaluate_result_txt_path):
        shutil.rmtree(variables.evaluate_result_txt_path)
        os.mkdir(variables.evaluate_result_txt_path)

    if os.listdir(variables.debug_kml_collection_path):
        shutil.rmtree(variables.debug_kml_collection_path)
        os.mkdir(variables.debug_kml_collection_path)

    if os.listdir(variables.problem_list_collection_path):
        shutil.rmtree(variables.problem_list_collection_path)
        os.mkdir(variables.problem_list_collection_path)

    # ------------------jenkins folder----------------
    if not os.path.exists(variables.jenkins_folder):
        os.mkdir(variables.jenkins_folder)
        os.mkdir(variables.storage_transfer_folder)

    if not os.path.exists(variables.storage_transfer_folder):
        os.mkdir(variables.storage_transfer_folder)
    elif os.listdir(variables.storage_transfer_folder):
        shutil.rmtree(variables.storage_transfer_folder)
        os.mkdir(variables.storage_transfer_folder)

    try:
        run_loc_cases.run_loc_by_diff_db(variables.rtv_files, variables.database_path, sys.argv[1])
        print(' run_loc_cases,successful!')
    except Exception as error:
        print(error)
        print('error:run_loc_cases!')
    
    #do kml_estimation
    try:

        kml_estimation.kml_estimation_func_by_diff_db(variables.kml_estimation_exe_path,variables.loc_result_path,variables.evaluate_result_txt_path)
    except Exception as error:
        pass
        
    #get optical flow result for all cases
    try:
        log_handle.get_optical_data_by_diff_db(variables.loc_result_path, variables.evaluate_result_txt_path)
    except Exception as error:
        pass

        
    try:
        compare.compare_result_by_diff_db(variables.loc_result_path, variables.case_db_reference_path,
                                          variables.compare_resut_txt_path, variables.code_path, variables.temp_folder)
        print(' compare,successful!')
    except Exception as error:
        print('error:compare!')

    try:
        evaluate50.evaluate_all(variables.compare_resut_txt_path, variables.evaluate_result_txt_path)
        print(' evaluate50,successful!')
    except Exception as error:
        print(error)  # ('error:evaluate50!')
    try:
        shutil.copytree(variables.evaluate_result_txt_path,
                        variables.storage_transfer_folder + '/' + variables.evaluate_result_txt_path.split('/')[-1])
        shutil.copytree(variables.debug_kml_collection_path,
                        variables.storage_transfer_folder + '/' + variables.debug_kml_collection_path.split('/')[-1])
        shutil.copytree(variables.problem_list_collection_path,
                        variables.storage_transfer_folder + '/' + variables.problem_list_collection_path.split('/')[-1])
    except Exception as error:
        pass

    try:
        os.chdir('/media/psf/Untitled/auto_loc_workspace/jenkins')
        shutil.make_archive('evaluate_kml_problemlist_folder', 'zip','/media/psf/Untitled/auto_loc_workspace/jenkins/storage_transfer_folder')
    except Exception as error:
        pass

    try:
        task_name=variables.parent_path.split('/')[-1]
        all_result_folder = variables.loc_result_path.split(variables.loc_result_path.split('/')[-1])[0] +'all_result_' + datetime.now().strftime('%y-%m-%d_%I:%M:%S_%p')
        os.mkdir(all_result_folder)
        shutil.move(variables.loc_result_path, all_result_folder)
        shutil.move(variables.compare_resut_txt_path, all_result_folder)
        shutil.move(variables.evaluate_result_txt_path, all_result_folder)
        shutil.move(variables.debug_kml_collection_path, all_result_folder)
        shutil.move(variables.problem_list_collection_path, all_result_folder)
        if not os.path.exists(variables.show_task_folder_path):
            os.mkdir(variables.show_task_folder_path)
        task_name_folder=variables.show_task_folder_path + '/' + task_name
        if not os.path.exists(task_name_folder):
            os.mkdir(task_name_folder)
        shutil.move(all_result_folder,task_name_folder)
    except Exception as error:
        pass
    try:
	os.chdir(task_name_folder)
	shutil.make_archive(all_result_folder.split('/')[-1], 'zip',task_name_folder + '/' + all_result_folder.split('/')[-1])
	kml_zip_name='debug_kml_collection_'+all_result_folder.split('/')[-1].split('all_result_')[1]
        shutil.make_archive(kml_zip_name, 'zip',task_name_folder + '/' + all_result_folder.split('/')[-1]+'/debug_kml_collection')
	zip_download_path=task_name_folder + '/' + all_result_folder.split('/')[-1]+'/zip_to_download'
	if not os.path.exists(zip_download_path):
	    os.mkdir(zip_download_path)
        shutil.move(task_name_folder + '/'+all_result_folder.split('/')[-1]+'.zip', zip_download_path)
        shutil.move(task_name_folder + '/'+kml_zip_name+'.zip', zip_download_path)
    except Exception as error:
        pass

    os.mkdir(variables.loc_result_path)
    os.mkdir(variables.compare_resut_txt_path)
    os.mkdir(variables.evaluate_result_txt_path)
    os.mkdir(variables.debug_kml_collection_path)
    os.mkdir(variables.problem_list_collection_path)
