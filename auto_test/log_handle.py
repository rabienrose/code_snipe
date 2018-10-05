# -*- coding: utf-8 -*-
import os
import variables
import string
import shutil
from datetime import datetime

def log_handle(log_txt_dir_path,config_num):
    log_txt_dir_list=os.listdir(log_txt_dir_path)
    log_txt_path=''
    for file_name in log_txt_dir_list:
        if 'log.txt_' in file_name:
            log_txt_path=os.path.join(log_txt_dir_path,file_name)
            break
    if os.path.exists(log_txt_path):
        kf_success_str_l = 'kf-success='
        kf_success_str_r = ', mp-success='
        kf_success_frmid_l='mp-success='
        kf_success_frmid_r=' (SlamDbgSnippetLoader.cpp'

        candikfs_str_l = 'locating, candikfs [num='
        candikfs_str_r = '], current rID'
        candikfs_frmid_l = 'at frmId:'
        candikfs_frmid_r = ' (Localization.cpp'

        candiMps_str_l = 'locating, candiMps [num='
        candiMps_str_r = '] at frmId'
        candiMps_frmid_l = 'at frmId:'
        candiMps_frmid_r = ' (Localization.cpp'

        locate_matched_points_number_str_l = 'locate matched points number:'
        locate_matched_points_number_str_r = ' at frmId'
        locate_matched_points_number_frmid_l = 'at frmId:'
        locate_matched_points_number_frmid_r = ' (Localization.cpp'

        locating_solving_pnp_dist_str_l = ',dist:'
        locating_solving_pnp_dist_str_r = ',estimated'
        locating_solving_pnp_dist_frmid_l = 'locating solving pnp, frmid:'
        locating_solving_pnp_dist_frmid_r = ',dist'


        check_pnp_smoothness_failed_str_l = 'check pnp smoothness f'
        check_pnp_smoothness_failed_str_r = '. (Localization.cpp'
        check_pnp_smoothness_failed_frmid_l = 'frmidx:'
        check_pnp_smoothness_failed_frmid_r = ' check pnp smoothness f'

        str_l_list = [kf_success_str_l, candikfs_str_l, candiMps_str_l,locate_matched_points_number_str_l,locating_solving_pnp_dist_str_l,check_pnp_smoothness_failed_str_l]
        str_r_list = [kf_success_str_r, candikfs_str_r, candiMps_str_r,locate_matched_points_number_str_r,locating_solving_pnp_dist_str_r,check_pnp_smoothness_failed_str_r]
        frmid_l_list = [kf_success_frmid_l, candikfs_frmid_l, candiMps_frmid_l,locate_matched_points_number_frmid_l,locating_solving_pnp_dist_frmid_l,check_pnp_smoothness_failed_frmid_l]
        frmid_r_list = [kf_success_frmid_r, candikfs_frmid_r, candiMps_frmid_r,locate_matched_points_number_frmid_r,locating_solving_pnp_dist_frmid_r,check_pnp_smoothness_failed_frmid_r]

        kf_success = {}
        candikfs = {}
        candiMps = {}
        locate_matched_points_number = {}
        locating_solving_pnp_dist = {}
        check_pnp_smoothness_failed = {}
        list_all = [kf_success, candikfs, candiMps,locate_matched_points_number,locating_solving_pnp_dist,check_pnp_smoothness_failed]

        problem_list = []
        print 'problem_list'
        print problem_list
        with open(log_txt_path, 'r') as log_txt:

            for line in log_txt.readlines():
                i = 0
                for str_l in str_l_list:
                    if str_l in line:
                        frmid = line.split(frmid_l_list[i])[1].split(frmid_r_list[i])[0]
                        value = line.split(str_l_list[i])[1].split(str_r_list[i])[0]
                        #print str_l + '___i:'+str(i)+ '            ' + value
                        list_all[i][frmid] = value
                    i += 1
        j=0
        for i in range(0, 2):
            if list_all[i].items():
                for num in set(list_all[i].values()):
                    if int(num) != 0:
                        j=1
                        break
            problem_list.insert(i, j)

        if problem_list[1]==1:
            for frmid, value in candikfs.items():
                #print 'value:'+value
                if value != 0:
                    try:
                        value_candiMps=candiMps[frmid]
                        #print 'value_candiMps:'+value_candiMps
                        if value_candiMps == 0:
                            problem_list.insert(2, 0)
                            break
                    except Exception as error:
                        print error
                        print('cant find the frmid: '+frmid+' in candiMps_dic!')
            problem_list.insert(2,1)
        else:
            problem_list.insert(2,-1)
        #print problem_list[2]

        x=0
        y=0
        print candiMps.items()
        print locate_matched_points_number.items()
        for frmid,value_a in candiMps.items():
            if int(value_a) != 0:
                try:
                    value_b = locate_matched_points_number[frmid]
                    c=float(value_b) / float(value_a)
                    if c < float(config_num):
                        x+=1
                    else:
                        y+=1
                except Exception as error:
                    pass
                    #print error
                    #print('cant find the frmid: ' + frmid + ' in locate_matched_points_number_dic!')

        try:
            if x/(x+y)>0.5:
                problem_list.insert(3,0)
            else:
                problem_list.insert(3,1)
        except Exception as error:
            print error
            problem_list.insert(3,-1)
        print problem_list[3]

        sgment=[]
        key_list_no_sort=locating_solving_pnp_dist.keys()
        new_numbers = []
        for n in key_list_no_sort:
            new_numbers.append(int(n))
        key_list = sorted(new_numbers)

        for i in range(0,len(key_list)-2):
            if key_list[i+1]-key_list[i]>=100:
                sgment.append(key_list[i])
        sgment.append(key_list[-1])
        print sgment

        dist_count_1=0
        dist_count_2=0
        failed_num=0
        db_quality=1
        db_quality_list=[]
        for s in sgment:
            for frmid in key_list:
                if frmid <= s:
                    dist_value=float(locating_solving_pnp_dist[str(frmid)])
                    #print dist_value
                    if dist_value <= 20:
                        dist_count_1 += 1
                    else:
                        dist_count_2 += 1
                    try:
                        f=locate_matched_points_number[str(frmid)]
                        failed_num+=1
                    except Exception as error:
                        print('ignore')

            if float((dist_count_2 + failed_num))/float(dist_count_1) > 0.2:
                db_quality=0

            else:
                db_quality=1

            db_quality_list.append(key_list[0])
            db_quality_list.append(s)
            db_quality_list.append(db_quality)
            i=0
            while i!=1:
                if key_list and key_list[i]<=s:
                    key_list.remove(key_list[i])
                else:
                    i=1
        print db_quality_list

        with open(log_txt_dir_path+'/problem_list_'+datetime.now().strftime('%y-%m-%d_%I:%M:%S_%p')+'.txt', 'w') as problem_list_txt:
            problem_list_txt.write('-----------problem list-----------'+'\n')
            if problem_list[0]==0:
                #print problem_list[0]
                problem_list_txt.write('deserialization failuer!'+'\n')
                problem_list_txt.write('\n')
            if problem_list[1]==0:
                #print problem_list[1]
                problem_list_txt.write('case dosen\'t match db!'+'\n')
                problem_list_txt.write('\n')
            if problem_list[2]==0:
                #print problem_list[2]
                problem_list_txt.write('db format is wrong!'+'\n')
                problem_list_txt.write('\n')
            if problem_list[3]==0:
                #print problem_list[3]
                problem_list_txt.write('db descriptor doesn\'t match loc!!'+'\n')
                problem_list_txt.write('\n')

            seg_num=len(db_quality_list)/3
            index=0
            for s in range(1,seg_num+1):
                if(db_quality_list[index+2]==0):
                    print db_quality_list[index+2]
                    problem_list_txt.write('sgement:'+str(s)+'\n')
                    problem_list_txt.write('start_frame:'+str(db_quality_list[index])+' ')
                    problem_list_txt.write('end_frame:'+str(db_quality_list[index+1])+'\n')
                    problem_list_txt.write('db_quality:so bad!'+'\n')
                    problem_list_txt.write('\n')
                index+=3
	    if(seg_num >= 1):
	    	problem_list_txt.write('what db quality so bad means:'+'\n')
		problem_list_txt.write('搜索所有的locating solving pnp, frmid:711,dist:, 记录下frmId和dist值; 搜索所有的check pnp smoothness failed，记录下frmId;'+'\n')
		problem_list_txt.write('如果发现出现（dist值超过20 + failed的次数）  /  dist小于20的次数  > 20%，表示db质量很差，如果帧号间隔相差100帧,则重新统计')

def loc_output_files_collection(caseName,db_folder,loc_case_output_path,debug_kml_collection_path,problem_list_collection_path):
    loc_case_output_folder_list=os.listdir(loc_case_output_path+'/output')
    for file_name in loc_case_output_folder_list:
        if 'debug' in file_name:
            debug_kml_path=os.path.join(loc_case_output_path+'/output', file_name)
            print debug_kml_collection_path
            shutil.copy(debug_kml_path,debug_kml_collection_path+'/db:'+db_folder+'_case:'+file_name)
    loc_case_output_list = os.listdir(loc_case_output_path)
    for file_name in loc_case_output_list:
        if 'problem_list' in file_name:
            problem_list_txt_path = os.path.join(loc_case_output_path, file_name)
            shutil.copy(problem_list_txt_path,
                        problem_list_collection_path + '/case:' + caseName + '_db:' + db_folder + '_problem_list.txt')
            break


def get_optical_data(result_path_single_db, db_name, output_file_path):
    """
    Caculate total 2d track count and total 3d track count for all test cases and write result to result file
    :param result_path: It saves all cases result in this folder
    :return: none . It will create a csv file to save optical flow result.
            The result includes total 3d track count, total 2d track count, average value of EPI drop and MSE drop
            for each case
    """
    loc_logfile = []
    epi_drop = []
    mse_drop = []
    #delte exsists result output file
    try:
        os.remove(output_file_path+"/"+"optical_flow.csv")
    except OSError:
        pass
    
    #generate optical flow result output file heaer
    with open(output_file_path+"/"+"optical_flow.csv",'w') as opt_result:
        opt_result.write("CASE_NAME_WITH_DB"+","+"TOTAL_3D_TRACK"+","+"TOTAL_2D_TRACK"+","+"AVERAGE_FOR_EPI_DROP"+","+"AVERAGE_FOR_MSE_DROP"+"\n")
    
    #get all cases log files absolute path and save them to list loc_logfile      
    for root,dir,file in os.walk(result_path_single_db):
        if 'log' in str(file):
            loc_logfile.append(root[:]+'/'+file[0])
    
    #get all result data and save it to result file
    for log in loc_logfile:
        case_name = str(log).split('/')[-2]
        with open(log,'r') as log_result:
            for line in log_result:
                if variables.track_2d_count_flag in line:
                    track_2d_count = int(line.split(variables.track_2d_count_flag)[1].split(',')[0].strip())
                    track_3d_count = int(line.split(variables.track_3d_count_flag)[1].split('(Localization.cpp::597)')[0].strip())
                if variables.epi_count_flag in line:
                    epi_drop.append(int(line.split(variables.epi_count_flag)[1].split(',')[0]))
                if variables.mse_count_flag in line:
                    mse_drop.append(int(line.split(variables.mse_count_flag)[1].split(',')[0]))
        avrg_epi_drop = (sum(epi_drop)/len(epi_drop))
        avrg_mse_drop = (sum(mse_drop)/len(mse_drop))

        with open(output_file_path+"/"+"optical_flow.csv",'a') as opt_result:
            opt_result.write(case_name+"_"+"DB_"+db_name.split('/')[-1]+","+str(track_3d_count)+","+str(track_2d_count)+","+str(avrg_epi_drop)+","+str(avrg_mse_drop)+"\n")


def get_optical_data_by_diff_db(loc_result_path,output_file_path):
    loc_result_list=os.listdir(loc_result_path)
    for name in loc_result_list:
        result_path_single_db=os.path.join(loc_result_path,name)
        get_optical_data(result_path_single_db,name,output_file_path)
