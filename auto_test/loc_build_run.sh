#!/bin/bash
sudo killall -9 loc_realtime
run_loc_cmd=1 # 1:develop version  0:release version

frameworkBranch=feature/20180205_loc_dev_v1.0
commonApiBranch=feature/20180205_loc_dev_v1.0
globalCommonBranch=feature/20180205_loc_dev_v1.0
rdbCoreCommonBranch=feature/20180205_loc_dev_v1.0
algoCommonBranch=feature/RDB-24577_develop-for-release-2.3.1-DB
algoVehicleBranch=feature/20180205_loc_dev_v1.0

download_res=0 #Download the RTV, DB and Ref

rtv_list=( \
/only_for_test/test_case/rtv \
)
db_list=( \
/only_for_test/test_case/db \
)
reference_list=( \
/only_for_test/test_case/ref \
)

sftp_ip=10.69.6.12
sftp_account=roadDB
sftp_pw=WuiIBTKLjCglwh

task_name=dev_daily_output

new_repo=0 #Use repo to clone the whole code
compile_all=1 #Compile all the code except the algorithm_localization 
compile_loc=1

#==================================#
#Do not modify the content below
#==================================#
re_output_ftp=/upload/loc_re
workspace=/media/psf/Untitled/auto_loc_workspace
taskspace=${workspace}/${task_name}
taskspace_code=${taskspace}/code
taskspace_data=${taskspace}/data_center
taskspace_script=${taskspace}/py_script
if ! [ -d ${taskspace} ]; then
    mkdir ${taskspace}
fi
if ! [ -d ${taskspace_code} ]; then
    mkdir ${taskspace_code}
fi
if ! [ -d ${taskspace_data} ]; then
    mkdir ${taskspace_data}
fi
if [ -d ${taskspace_script} ]; then
    rm -rf ${taskspace_script}
fi

if (( ${new_repo} == 1 )); then
    echo "get new repo"
    rm -rf ${taskspace_code}
    mkdir ${taskspace_code}
    cd ${taskspace_code}
    git clone ssh://git@stash.ygomi.com:7999/as/3rdParty 
    git clone ssh://git@stash.ygomi.com:7999/as/common
    mkdir core
    mkdir framework
    cd ${taskspace_code}/core
    git clone ssh://git@stash.ygomi.com:7999/rc/algorithm_common
    git clone ssh://git@stash.ygomi.com:7999/rc/algorithm_vehicle_localization
    git clone ssh://git@stash.ygomi.com:7999/rc/common
    cd ${taskspace_code}/framework
    mkdir device
    cd device
    git clone ssh://git@stash.ygomi.com:7999/as/gmock
    git clone ssh://git@stash.ygomi.com:7999/as/roaddb_logger
    git clone ssh://git@stash.ygomi.com:7999/as/roaddb_video
    git clone ssh://git@stash.ygomi.com:7999/as/road_in_vehicle_common_api
    #PATH=~/bin:$PATH && repo init -u ssh://git@stash.ygomi.com:7999/as/manifest.git -b feature/loc_dev -g loc && repo sync
fi
if (( ${compile_all} == 1 )); then
    cd ${taskspace_code}
    rootDir=${taskspace_code}
    export CUDA_PATH=/usr/local/cuda-8.0
    echo -e "\n--------------------checkout 3rd party-------------------------!\n"
    cd "$rootDir"/3rdparty/jsoncpp && git fetch --all && git reset --hard &&  git checkout $frameworkBranch && git pull 
    if [ $? -ne 0 ];then 
       echo -e "\ncheckout 3rd party failed!\n"
       exit 1
    fi
    echo -e "\n--------------------checkout global common-------------------------!\n"
    cd "$rootDir"/common/ && git fetch --all && git reset --hard && git checkout $globalCommonBranch && git pull  
    if [ $? -ne 0 ];then 
       echo -e "\ncheckout global common failed!\n"
       exit 1
    fi
    echo -e "\n--------------------checkout framework-gmock-------------------------!\n"
    cd "$rootDir"/framework/device/gmock/ &&  git fetch --all && git reset --hard &&  git checkout $frameworkBranch && git pull  
    if [ $? -ne 0 ];then 
       echo -e "\ncheckout framework-gmock failed!\n"
       exit 1
    fi
    echo -e "\n--------------------checkout framework-roaddb-logger-------------------------!\n"
    cd "$rootDir"/framework/device/roaddb_logger/ &&  git fetch --all && git reset --hard &&  git checkout $frameworkBranch && git pull  
    if [ $? -ne 0 ];then 
       echo -e "\ncheckout framework-roaddb-logger failed!\n"
       exit 1
    fi
    echo -e "\n--------------------checkout framework-roaddb-video-------------------------!\n"
    cd "$rootDir"/framework/device/roaddb_video/ &&  git fetch --all && git reset --hard &&  git checkout $frameworkBranch && git pull  
    if [ $? -ne 0 ];then 
       echo -e "\ncheckout framework-roaddb-video failed!\n"
       exit 1
    fi
    echo -e "\n--------------------checkout common api-------------------------!\n"
    cd "$rootDir"/framework/device//road_in_vehicle_common_api/ &&  git fetch --all && git reset --hard &&  git checkout $frameworkBranch && git pull 
    if [ $? -ne 0 ];then 
       echo -e "\nbuild global common failed!\n"
       exit 1
    fi
    echo -e "\n--------------------checkout core common-------------------------!\n"
    cd "$rootDir"/core/common/ && git fetch --all && git reset --hard && git checkout $rdbCoreCommonBranch && git pull  
    if [ $? -ne 0 ];then 
       echo -e "\ncheckout core common failed!\n"
       exit 1
    fi
    echo -e "\n--------------------checkout algorithm_common-------------------------!\n"
    cd "$rootDir"/core/algorithm_common/ && git fetch --all && git reset --hard &&  git checkout $algoCommonBranch && git pull
    if [ $? -ne 0 ];then 
       echo -e "\ncheckout algorithm_common failed!\n"
       exit 1
    fi
    echo -e "\n--------------------checkout algorithm_vehicle_localization-------------------------!\n"
    cd "$rootDir"/core/algorithm_vehicle_localization/ && git fetch --all && git reset --hard &&  git checkout $algoVehicleBranch && git pull
    if [ $? -ne 0 ];then 
       echo -e "\ncheckout algorithm_vehicle_localization failed!\n"
       exit 1
    fi
    echo -e "\n--------------------build 3rd party-------------------------!\n"
    cd "$rootDir"/3rdparty  && ./build.sh
    if [ $? -ne 0 ];then 
       echo -e "\nbuild 3rd party failed!\n"
       exit 1
    fi
    echo -e "\n--------------------build global common-------------------------!\n"
    cd "$rootDir"/common/ && ./build.sh
    if [ $? -ne 0 ];then 
       echo -e "\nbuild global common failed!\n"
       exit 1
    fi
    echo -e "\n--------------------build framework-gmock-------------------------!\n"
    cd "$rootDir"/framework/device/gmock/  && ./build.sh
    if [ $? -ne 0 ];then 
       echo -e "\nbuild framework-gmock failed!\n"
       exit 1
    fi
    echo -e "\n--------------------build framework-roaddb-logger-------------------------!\n"
    cd "$rootDir"/framework/device/roaddb_logger/  && ./build.sh
    if [ $? -ne 0 ];then 
       echo -e "\nbuild framework-roaddb-logger failed!\n"
       exit 1
    fi
    echo -e "\n--------------------build framework-roaddb-video-------------------------!\n"
    cd "$rootDir"/framework/device/roaddb_video/  && ./build.sh
    if [ $? -ne 0 ];then 
       echo -e "\nbuild framework-roaddb-video failed!\n"
       exit 1
    fi
    echo -e "\n--------------------build core common-------------------------!\n"
    cd "$rootDir"/core/common/ && ./build.sh
    if [ $? -ne 0 ];then 
       echo -e "\nbuild core common failed!\n"
       exit 1
    fi
    echo -e "\n--------------------build common api-------------------------!\n"
    cd "$rootDir"/framework/device/road_in_vehicle_common_api/ && ./build.sh
    if [ $? -ne 0 ];then 
       echo -e "\nbuild common api failed!\n"
       exit 1
    fi
    echo -e "\n--------------------build core algorithm_common-------------------------!\n"
    cd "$rootDir"/core/algorithm_common/ && ./build.sh
    if [ $? -ne 0 ];then 
       echo -e "\nbuild algorithm_common failed!\n"
       exit 1
    fi
   
fi


if (( ${compile_loc} == 1 )); then
    echo -e "\n--------------------build core algorithm_Vehicle-------------------------!\n"
    cd ${taskspace_code}/core/algorithm_vehicle_localization/ && ./build.sh
    if [ $? -ne 0 ];then 
        echo -e "\nbuild algorithm_vehicle_localization failed!\n"
        exit 1
    fi
    
    echo -e "\n--------------------build kml_estimation-------------------------!\n"
    cd ${taskspace_code}/core/algorithm_vehicle_localization/examples/db_est_tool
    if [ -d build ]; then
    	rm -rf build
    fi
    
    mkdir build && cd build && cmake .. && make
    
    if [ $? -ne 0 ];then 
        echo -e "\nbuild kml_estimation failed!\n"
        exit 1
    fi
fi

config_file_name='locConfigRosViewer.json'
config_para='EnableKmlSave'
modify_value='1'

#python ${taskspace_script}/config_modify.py ${config_file_name} ${config_para} ${modify_value}

if (( ${download_res} == 1 )); then
    cd ${taskspace_data}
    rm -rf ${taskspace_data}/*
    mkdir input
    rtv_root=${taskspace_data}/input/rtv
    db_root=${taskspace_data}/input/db
    ref_root=${taskspace_data}/input/reference
    mkdir ${rtv_root}
    mkdir ${db_root}
    mkdir ${ref_root}
    for db_item in "${db_list[@]}"
    do
        sshpass -p "${sftp_pw}" sftp -r -oPort=22 -o StrictHostKeyChecking=no ${sftp_account}@${sftp_ip}:${db_item}/* ${db_root}
    done
    for rtv_item in "${rtv_list[@]}"
    do
        sshpass -p "${sftp_pw}" sftp -r -oPort=22 -o StrictHostKeyChecking=no ${sftp_account}@${sftp_ip}:${rtv_item}/* ${rtv_root}
    done
    for reference_item in "${reference_list[@]}"
    do
        sshpass -p "${sftp_pw}" sftp -r -oPort=22 -o StrictHostKeyChecking=no ${sftp_account}@${sftp_ip}:${reference_item}/* ${ref_root}
    done
fi

cd ${taskspace}
git clone ssh://git@stash.ygomi.com:7999/rc/testcase.git -b auto_py_script py_script
cd ${taskspace_script}
python run_local_compare_evaluate.py ${run_loc_cmd}
