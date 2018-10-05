# -*- coding: utf-8 -*-
import os
import subprocess
import variables
import time
import log_handle


def generate_rtv_command(rtv_file, imu_file, gps_file, db_file,run_loc_cmd):
    """
    :param rtv_file:rtv file name with absolute path
    :param imu_file: imu file name with absolute path
    :param gps_file: gps file name with absolute path
    :return:Localization command for running rtv file
    """
    if int(run_loc_cmd)==1:
        return variables.code_path + '/' + 'algorithm_vehicle_localization/examples/LocRealTime/loc_realtime' + ' ' + \
               '-c' + ' ' + \
               variables.code_path + '/' + 'algorithm_vehicle_localization/config/camera_65_hybrid_lens.json' + ' ' + \
               '-f' + ' ' + \
               variables.code_path + '/' + 'algorithm_vehicle_localization/config/slamConfig_loc.json' + ' ' + \
               '-d' + ' ' + \
               db_file + ' ' + \
               '-t' + ' ' + \
               rtv_file + ' ' + \
               '-s' + ' ' + \
               variables.rtv_start_frame + ' ' + \
               '-r' + ' ' + \
               variables.rtv_total_frames + ' ' + \
               '-v' + ' ' + \
               variables.bow_file + ' ' + \
               '-l' + ' ' + \
               variables.code_path + '/' + 'algorithm_vehicle_localization/config/locConfigRosViewer.json' + ' ' + \
               '-i' + ' ' + \
               variables.code_path + '/' + 'algorithm_vehicle_localization/config/IMUConfig_Conti65.json' + ' ' + \
               '-m' + ' ' + \
               imu_file + ' ' + \
               '-e' + ' ' + '0' + ' ' + \
               '--ologpath' + ' ' + './log.txt' + \
               ' -o ./output'
    else:
        return variables.code_path + '/' + 'algorithm_vehicle_localization/examples/LocRealTime/loc_realtime' + ' ' + \
               variables.code_path + '/' + 'algorithm_vehicle_localization/config/camera_65_hybrid_lens.json' + ' ' + \
               variables.code_path + '/' + 'algorithm_vehicle_localization/config/slamConfig_loc.json' + ' ' + \
               db_file + ' ' + \
               rtv_file + ' ' + \
               gps_file + ' ' + \
               variables.rtv_start_frame + ' ' + \
               variables.rtv_total_frames + ' ' + \
               variables.bow_file + ' ' + \
               variables.code_path + '/' + 'algorithm_vehicle_localization/config/locConfigRosViewer.json' + ' ' + \
               variables.code_path + '/' + 'algorithm_vehicle_localization/config/IMUConfig_Conti65.json' + ' ' + \
               imu_file

def generate_ros_command():
    """
    :return: Localization command for running ros bag file
    """
    return variables.code_path + '/' + \
           'algorithm_vehicle_localization/examples/LocRealTimeRosAlgo/LocRealTimeRosAlgo' + ' ' + \
           '-c' + ' ' + \
           variables.code_path + '/' + 'algorithm_vehicle_localization/config/camera_loc.json' + ' ' + \
           '-f' + ' ' + \
           variables.code_path + '/' + 'algorithm_vehicle_localization/config/slamConfig_loc.json' + ' ' + \
           '-d' + ' ' + \
           variables.database_path + ' ' + \
           '-s' + ' ' + \
           variables.ros_start_time + ' ' + \
           '-r' + ' ' + \
           variables.ros_total_time + ' ' + \
           '-v' + ' ' + \
           variables.bow_file + ' ' + \
           '-l' + ' ' + \
           variables.code_path + '/' + 'algorithm_vehicle_localization/config/locConfigRosViewer.json' + ' ' + \
           '-i' + ' ' + \
            variables.code_path + '/' + 'algorithm_vehicle_localization/config/IMUConfig_Conti65.json' + ' ' + \
            '-t' + ' ' + 'live' + ' ' + \
            '-g' + ' ' + 'ologpath' + ' ' + '&'


def execute_rtv_cases(rtv_files, db_file_path,run_loc_cmd):
    """
    This function could execute all rtv cases in data_path
    :param data_path: absolute path for rtv folders
    :return: none
    """
    db_folder = db_file_path.split('/')[-1]
    print db_folder
    if(os.path.exists(variables.loc_result_path + '/' + db_folder)):
        print('stoped by already existed:'+variables.loc_result_path + '/' + db_folder)
        return
    else:
        os.mkdir(variables.loc_result_path + '/' + db_folder)

    for root, dirs, files in os.walk(rtv_files):
        for case in files:
            if '.imu' in case:
                imu_file = os.path.join(root, case)
                print imu_file
                caseName = case.split('GMT')[0]+'GMT'+case.split('GMT')[1].split('.')[0]
                # print 'caseName:'+caseName
                rtv_file = os.path.join(root, caseName + '.rtv')
                print rtv_file
                gps_file = os.path.join(root, caseName + '.gps')
                print gps_file
                try:
                    loc_case_output=variables.loc_result_path + '/' +db_folder+'/'+caseName
                    os.mkdir(loc_case_output)
                    os.chdir(loc_case_output)
                    os.system(generate_rtv_command(rtv_file, imu_file, gps_file, db_file_path,run_loc_cmd))
                except Exception as error:
                    pass
                try:
                    log_handle.log_handle(loc_case_output, 0.1)
                except Exception as error:
                    pass
                try:
                    log_handle.loc_output_files_collection(caseName,db_folder,loc_case_output,variables.debug_kml_collection_path,variables.problem_list_collection_path)
                except Exception as error:
                    pass


    """
    :database_path is top path

    """
def run_loc_by_diff_db(case_path,database_path,run_loc_cmd):
    if (os.path.exists(database_path)):
        db_folders = os.listdir(database_path)
        for db_folder in db_folders:
            db_file_path = os.path.join(database_path,db_folder)
            if (os.path.isdir(db_file_path)):
                execute_rtv_cases(case_path, db_file_path,run_loc_cmd)
                print'db_path:'+(db_file_path)

def execute_ros_cases(data_path):
    """
    execute all rosbag files in data_path
    :param data_path: absolute path that saved rosbag files
    :return: none
    """
    print generate_ros_command()
    print "********************kill LocRealTimeRosAlgo process ********************"
    os.system('kill -9 ${pidof LocRealTimeRosAlgo}')
    print "*********************start LocRealTimeRosAlgo***************************"
    subprocess.call(generate_ros_command(), shell=True)
    for root, dir, file in os.walk(data_path):
        for subfile in file:
            if '.bag' in subfile:
                ros_file = os.path.join(root, subfile)
                case_result_folder = ros_file.split('/')[-1].split('.bag')[0]
                os.mkdir(variables.loc_result_path + '/' + case_result_folder)
            else:
                print "No ROS bag files"
                break
            os.chdir(variables.loc_result_path + '/' + case_result_folder)
            try:
                # print generate_ros_command()
                print ros_file
                print "$$$$$$$$$$$$$$$$$$$$$play rosbag file$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"

                os.system('rosbag play %s' % ros_file)
                # os.system('rosbag play %s' % variables.stop_rosbag)
                loc_process = subprocess.check_output('ps -ef | grep LocRealTimeRosAlgo', shell=True)
                time.sleep(3000)
                while ('LocRealTimeRosAlgo/LocRealTimeRosAlgo' in loc_process):
                    print "%%%%%%%%%%%%%%%%%%%%play stop rosbag%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
                    print loc_process
                    os.system('rosbag play %s' % variables.stop_rosbag)
                print "************************************************"
            except Exception as error:
                pass


