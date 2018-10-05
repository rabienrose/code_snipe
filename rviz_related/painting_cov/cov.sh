#!/bin/bash

ROSALGOSRV=/home/user/workspace/loc/core/algorithm_vehicle/examples/RosAlgoService/build/RosAlgoService
ROSCORE=/opt/ros/install_isolated/bin/roscore
PAINTING_COV=/home/user/workspace/loc/core/rdb-loc-visualization/ubuntu_demo/rviz_modify/dist/x64/algorithm_localisation/bin/painting_cov


killall RosAlgoService
killall roscore
gnome-terminal \
--tab --title="roscore" -e 'bash -c '$ROSCORE'; exec bash;”' \
--tab --title="RosAlgoService" -e 'bash -c '$ROSALGOSRV'; exec bash;”'
sleep 2

kmlfile=`find $1 -name '*.kml'`
for file in $kmlfile
do
    echo $file
    $PAINTING_COV $file | tee cov_log.txt | cat
    echo '--------------'
    echo ' '
done

killall RosAlgoService
killall roscore
#exit
