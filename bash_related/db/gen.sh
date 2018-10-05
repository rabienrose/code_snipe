#!/bin/bash
input_db=none

rtv_slam_root=/media/psf/Home/Documents/data/db_checker/slam
rtv_al_root=/media/psf/Home/Documents/data/db_checker/empty
source_code=/media/psf/Home/Documents/code/db
config_addr=${source_code}/core/vehicle/config
maplist_name=maplist.txt
inclist_name=inclist.txt
sam_output_name=output_sam

if [[ ${input_db} == "none" ]]; then
    for rtv_name in `ls ${rtv_slam_root}`
    do
        ${source_code}/core/algorithm_vehicle_slam/example/debug/algoSlamExe --ic ${config_addr}/camera65withDistort.json --ip ${config_addr}/slamConfig.json --ivg ${rtv_slam_root}/${rtv_name}/${rtv_name}.rtv --ivid 170ca9d4e6b40738 --osp sp.out/ --ort rt.out --idb ./ --ol ./log_slam --ivoc ${config_addr}/Highway_Detroit_Downtown_sum--0--1799-4_voc --tmp ./output/ --d ./output/ --iimu ${rtv_slam_root}/${rtv_name}/${rtv_name}.imu
    done
    
    if [ -f ${maplist_name} ]; then
       rm ${maplist_name}
    fi
    
    echo "#############start to create map list###################"
    
    for file_name in `ls ./output`
    do
        if [[ ${file_name} == *".rtv" ]]; then
            echo ${file_name}
            for snip_name in `ls ./output/${file_name}`
            do
                if [[ ${snip_name} == "SlamSnippet"* ]]; then
                    echo ${snip_name}
                    echo output/${file_name}/${snip_name} >> ${maplist_name}
                fi
            done
        fi
    done
    
    echo "##############start to run serverExampleSlam##############"
    ${source_code}/core/algorithm_sam/build/example/serverExampleSlam 1 . ./${sam_output_name}
fi

db_addr=./${sam_output_name}/section_out
for rtv_name in `ls ${rtv_al_root}`
do
	${source_code}/core/algorithm_vehicle_slam/example/debug/algoSlamExe --ic  ${config_addr}/camera65withDistort.json --ip ${config_addr}/slamConfig.json --ivg ${rtv_al_root}/${rtv_name}/${rtv_name}.rtv --ivid 170ca9d4e6b40738 --ort kf.out --osp sp.out/ --ort rt.out --idb ./ --dso ${db_addr} --ivoc ${config_addr}/Highway_Detroit_Downtown_sum--0--1799-4_voc --tmp ./output/ --d ./output/ --iimu ${rtv_al_root}/${rtv_name}/${rtv_name}.imu
    if [ -f ${inclist_name} ]; then
       rm ${inclist_name}
    fi
    incSnip_addr=./output/${rtv_name}.rtv/incSnippet.bin
    if [ -f ${incSnip_addr} ]; then
        echo output/${rtv_name}.rtv/incSnippet.bin >> ${inclist_name}
        ${source_code}/core/algorithm_sam/build/example/serverExampleSlam 2 . ./${sam_output_name}
    fi
done
