ulimit -c unlimited
exe_addr=/media/psf/Home/Documents/code/dev/core/algorithm_vehicle_localization/examples/DBModifier/DBModifier
config_adddr=/media/psf/Home/Documents/code/dev/core/algorithm_vehicle_localization/config/slamConfig_DBCalDesc.json
db_addr=/media/psf/Home/Documents/data/ca_slam/short_1000_new
${exe_addr} ./bag_list.txt ${config_adddr} ${db_addr}

