#read me
#$ gedit .bashrc
#  add: eval "$BASH_POST_RC"
#!/bin/bash


if [ $# -lt 2 ];then
    echo "MUST give me ros ip and master ip!"
    echo "example : ./run.sh 127.0.0.1 127.0.0.1"
    exit 1
fi

unset ROS_IP
unset ROS_MASTER_URI
unset REMOTE_ACCOUNT
unset REMOTE_PASSWORD

if [ $1 != "127.0.0.1" ];then
    if [ $# -gt 2 ];then
        export REMOTE_ACCOUNT=$3 
        export REMOTE_PASSWORD=$4
    #else
        #echo "Need remote account/password!"
        #echo "example : ./run.sh 10.69.144.155 10.69.144.195 roaddb test1234"
    fi
fi

title1="roscore"
title2="rviz"
title3="loc_server"
title4="show_map"
title5="param"
#export ROS_IP=`ifconfig | grep -E "addr.*Bcast" | awk '{print $2}' | awk -F ":" '{print $2}'`
#export ROS_IP=`ifconfig |grep -E "inet addr:.*Bcast" | sed -n "s/ \+inet addr:\(.*\) Bcast:.*/\1/p"`
export ROS_IP=$1
export ROS_MASTER_URI=http://$2:11311
echo $ROS_IP
echo $ROS_MASTER_URI

basepath=$(cd `dirname $0`; pwd)
echo ${basepath}

default_rviz_cfg=${basepath}/../share/rviz_animated_view_controller/launch/default.rviz
use_rviz_cfg=/tmp/tmp_rviz

rm -rf ${use_rviz_cfg} 
mkdir -p ${use_rviz_cfg}
cp ${default_rviz_cfg} ${use_rviz_cfg}


basepath=${basepath} use_rviz_cfg=${use_rviz_cfg} gnome-terminal \
--maximize --tab --title=$title2 -e 'bash -c "echo xx1=$ROS_IP && echo xx2=$ROS_MASTER_URI && source ${basepath}/../setup.bash &&rosrun rviz rviz -d ${use_rviz_cfg}/default.rviz; exec bash"' \
--maximize --tab --title=$title3 -e 'bash -c "echo wait 2 second...&& source ${basepath}/../setup.bash && sleep 2 && loc_server; exec bash"' \
--maximize --tab --title=$title4 -e 'bash -c "source ${basepath}/../setup.bash &&show_map; exec bash"'

