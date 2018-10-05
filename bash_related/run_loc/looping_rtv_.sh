ulimit -c unlimited
data_path=/media/psf/Home/Documents/code/auto_space/chamo/data_center/input
exeu_path=/media/psf/Home/Documents/code/auto_space/chamo/code/core/algorithm_vehicle_localization

for db_name in `ls ${data_path}/db`
do
    db_addr=${data_path}/db/${db_name}
    echo "db path name: " ${db_addr}

    for rtv_name in `ls ${data_path}/rtv`
    do
        rtv_addr=${data_path}/rtv/${rtv_name}
        echo "rtv path name: " ${rtv_addr}
        
        rtv_file=${rtv_addr}/${rtv_name}.rtv
        imu_file=${rtv_addr}/${rtv_name}.imu
        echo "rtv rtv file name: " ${rtv_file}
        echo "rtv imu file name: " ${imu_file}

        ${exeu_path}/examples/LocRealTime/loc_realtime \
        -c ${exeu_path}/config/camera_65_hybrid_lens.json \
        -f ${exeu_path}/config/slamConfig_loc.json \
        -d ${db_addr} -t ${rtv_file} -m ${imu_file} \
        -s 0 -r 100000 \
        -v ${exeu_path}/config/merged_two_lanes_voc.bin \
        -l ${exeu_path}/config/locConfigRosViewer.json \
        -i ${exeu_path}/config/IMUConfig_Conti65.json \
        -e 2 --ologpath ./log.txt -o ./output
    done
done
