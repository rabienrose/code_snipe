ulimit -c unlimited
exe_addr=/media/psf/Home/Documents/code/dev/core/algorithm_vehicle_localization/examples/DBModifier/DBModifier
config_adddr=/media/psf/Home/Documents/code/dev/core/algorithm_vehicle_localization/config/slamConfig_loc.json
db_addr=/media/psf/Home/Documents/data/new_slam_db/db1
${exe_addr} ./bag_list_731.txt ${config_adddr} ${db_addr}

