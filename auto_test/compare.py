# -*- coding: utf-8 -*-
import os
import shutil
import variables
from datetime import datetime

def execute_compare_result(loc_result_single_db,db_reference,compareResultTxt,code_path,temp_folder):
    if (os.path.exists(temp_folder)):
        shutil.rmtree(temp_folder)
    os.mkdir(temp_folder)
    traj_collection_folder=temp_folder + '/' + 'traj_collection_folder'
    os.mkdir(traj_collection_folder)
    temp_db_reference = temp_folder + '/' + 'temp_db_reference'
    os.mkdir(temp_db_reference)
    temp_traj_txt = temp_folder + '/' + 'temp_traj_txt'
    os.mkdir(temp_traj_txt)

    for root, dirs, files in os.walk(loc_result_single_db):
       # print files
        for trajTxt in files:
            if 'traj' in trajTxt  :
                old_traj_path = os.path.join(root,trajTxt)
                print(old_traj_path)
                new_traj_path = traj_collection_folder+'/'+trajTxt
                print(new_traj_path)
                shutil.copyfile(old_traj_path, new_traj_path)


    traj_list = os.listdir(traj_collection_folder)
    db_reference_list=os.listdir(db_reference)
    for db_refe in db_reference_list:
        db_refe_path=os.path.join(db_reference,db_refe)
        print('db_refe_path:'+db_refe_path)
        for traj in traj_list:
            trajCaseName=traj.split('GMT')[0]+'GMT'
            if trajCaseName in db_refe:
                old_traj_path_1=os.path.join(traj_collection_folder, traj)
                new_traj_path_1=temp_traj_txt+'/'+traj
                shutil.copyfile(old_traj_path_1, new_traj_path_1)
        if (os.path.exists(temp_traj_txt)):
            if(os.listdir(temp_traj_txt)):
               for file in os.listdir(db_refe_path):
                   if 'Snippet' in file:
                       shutil.copyfile(os.path.join(db_refe_path,file), temp_db_reference+'/'+file)

               if(os.listdir(temp_db_reference)):
                   try:
                        compare_cmd='/media/psf/Untitled/auto_loc_workspace/tool/calLocErrorBySlamSnippet'+' '+temp_traj_txt+' '+temp_db_reference+' '+compareResultTxt+' '+'1000'
                        print(compare_cmd)
                        os.system(compare_cmd)
                   except Exception as error:
                        print('stoped by error:calLocErrorBySlamSnippet')
                        return
                   os.rename(compareResultTxt+'/compareResult.txt', compareResultTxt+'/case:'+db_refe+'_db:'+loc_result_single_db.split('/')[-1]+'_compare.txt')# +datetime.now().strftime('%y-%m-%d_%I:%M:%S_%p'))
               #shutil.copytree(temp_folder,'/media/psf/Untitled/DatacenterForAutoTest_20180328/tool_test_0330/cp_temp/'+db_refe)# +datetime.now().strftime('%y-%m-%d_%I:%M:%S_%p'))
               shutil.rmtree(temp_db_reference)
               shutil.rmtree(temp_traj_txt)
               os.mkdir(temp_db_reference)
               os.mkdir(temp_traj_txt)
    shutil.rmtree(temp_folder)

def  compare_result_by_diff_db(loc_result_path,db_reference,compareResultTxt,code_path,temp_folder):
    loc_result_list = os.listdir(loc_result_path)
    for loc_result_single_db_folder in loc_result_list:
        loc_result_single_db=os.path.join(loc_result_path,loc_result_single_db_folder)
        execute_compare_result(loc_result_single_db, db_reference, compareResultTxt, code_path,temp_folder)

'''

if __name__ == '__main__':

    #calLocErrorBySlamSnippet_path=variables.calLocErrorBySlamSnippet_path
    loc_result_path=variables.loc_result_path
    db_reference=variables.case_db_reference_path
    compareResultTxt=variables.compare_resut_txt_path
    code_path=variables.code_path
    temp_folder=variables.temp_folder
    compare_result_by_diff_db(loc_result_path, db_reference, compareResultTxt,code_path,temp_folder)
    #execute_ros_cases(variables.rosbag_files)

'''

